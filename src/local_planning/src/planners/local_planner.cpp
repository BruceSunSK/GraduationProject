#include "local_planning/planners/local_planner.h"


void LocalPlanner::InitParams(const LocalPlannerParams & params)
{
    params_ = params;
}

void LocalPlanner::SetMap(const Map::MultiMap::Ptr & map)
{
    map_ = map;
    flag_map_ = true;
}

void LocalPlanner::SetReferencePath(const Path::ReferencePath::Ptr & reference_path)
{
    reference_path_ = reference_path;
    flag_reference_path_ = true;
}

void LocalPlanner::SetVehicleState(const Vehicle::State::Ptr & vehicle_state)
{
    vehicle_state_ = vehicle_state;
    flag_vehicle_state_ = true;
}

bool LocalPlanner::Plan(LocalPlannerResult & result, std::string & error_msg)
{
    result_.Clear();
    error_msg.clear();

    // 0. 判断当前帧数据是否完整
    if (!IsDataReady())
    {
        error_msg = "LocalPlanner::Plan() failed: data not ready.";
        return false;
    }
    result_.timestamp = std::chrono::duration<double>(std::chrono::steady_clock::now().time_since_epoch()).count();
    result_.log << "============================================================\n"
                << "LocalPlanner::Plan() start.\n";


    // 1. 寻找车辆在参考线上的投影点 并 更新车辆信息
    // 传入的vehicle_state仅包含x, y, theta, kappa, v, w，此处添加上s, l
    Vehicle::State curr_veh_state = *vehicle_state_;
    auto [curr_veh_proj_point, curr_veh_proj_nearest_idx] = reference_path_->GetProjection(
        { curr_veh_state.pos.x, curr_veh_state.pos.y }, last_veh_proj_nearest_idx_);
    last_veh_proj_nearest_idx_ = curr_veh_proj_nearest_idx;
    result_.log << "reference_path_.size(): " << reference_path_->GetSize() << "\n"
                << "curr_veh_proj_nearest_idx: " << curr_veh_proj_nearest_idx << "\n";

    Path::PointXY veh_xy = { curr_veh_state.pos.x, curr_veh_state.pos.y };
    Path::PointSL veh_sl = Path::Utils::XYtoSL(veh_xy,
        { curr_veh_proj_point.x, curr_veh_proj_point.y }, curr_veh_proj_point.s, curr_veh_proj_point.theta);
    curr_veh_state.pos.s = veh_sl.s;
    curr_veh_state.pos.l = veh_sl.l;
    result_.log << "curr_veh_state:\n" << curr_veh_state;

    // 2. 确定规划起点 
    Path::PathNode planning_start_point = GetPlanningStart(curr_veh_state, curr_veh_proj_point);
    result_.log << "planning_start_point:\n" << planning_start_point;
    
    // 3. 截断参考线
    // 本质上应该是获得一组采样的s点，不应该直接截断，会出很多问题。采样的点仅在车辆当前位置前后附近，不是全部参考线采样
    // reference_path_: 完整的参考线类型，内部会包含整条参考线及其各个点
    // ref_points: 在参考线上，只在车辆附近采样得到的点
    const auto ref_points = SampleReferencePoints(planning_start_point);

    // 3. 决策
    // todo

    // 4. 确定路径规划的上下边界
    // 包括两部分：①代价地图确定的边界；②决策部分给出的每辆车的边界。两部分每个点都是三碰撞圆的边界，两部分求交集得到最终上下边界。
    // 对于起点和终点，三碰撞圆也许会超出参考线的范围，没事，因为：参考线使用三次样条，能够外推一部分距离，够用了
    auto map_bounds = GetBoundsByMap(ref_points);

    // 5. 进行路径规划，QP优化
    std::vector<Path::PointXY> optimized_path;
    if (!PathQP(ref_points, map_bounds, planning_start_point, optimized_path))
    {
        error_msg = "LocalPlanner::Plan() failed: PathQP failed.";
        return false;
    }

    // 6. 转换为TrajectoryPoint类型
    for (const auto & point : optimized_path)
    {
        Path::TrajectoryPoint traj_point;
        traj_point.x = point.x;
        traj_point.y = point.y;
        traj_point.t = 0.0;
        traj_point.v = 0.0;
        traj_point.a = 0.0;
        traj_point.j = 0.0;
        result_.trajectory.push_back(traj_point);
    }

    auto end_time = std::chrono::duration<double>(std::chrono::steady_clock::now().time_since_epoch()).count();
    result_.planning_time = end_time - result_.timestamp;
    result_.log << "LocalPlanner::Plan() end.\n"
                << "LocalPlanner::Plan() elapsed time: " << result_.planning_time * 1000.0 << " ms.\n";
    result = std::move(result_);
    return true;
}


bool LocalPlanner::IsDataReady() const
{
    return flag_map_ && flag_reference_path_ && flag_vehicle_state_;
}

Path::PathNode LocalPlanner::GetPlanningStart(const Vehicle::State & curr_veh_state, const Path::PathNode & veh_proj_point)
{
    // 1. 车速为0，直接认为当前车辆位置就是规划起点情形
    // todo 上一帧无规划结果
    if (curr_veh_state.v < params_.start_point.V_TOLERANCE &&
        curr_veh_state.w < params_.start_point.W_TOLERANCE)
    {
        result_.log << "GetPlanningStart(): vehicle is stopped.\n";
        return curr_veh_state.pos;
    }

    // 2. 当前位置与上一帧规划结果偏移过大（控制没跟上），则使用运动学外推
    // todo 目前用l距离偏差简单代替
    if (curr_veh_state.pos.l > params_.start_point.POS_TOLERANCE)
    {
        Path::PathNode ret;
        const double w = curr_veh_state.w;
        const double v = curr_veh_state.v;
        const double dt = params_.common.PLANNING_CYCLE_TIME;
        if (std::abs(w) < 1e-6)
        {
            // 直线运动
            ret.x = curr_veh_state.pos.x + v * std::cos(curr_veh_state.pos.theta) * dt;
            ret.y = curr_veh_state.pos.y + v * std::sin(curr_veh_state.pos.theta) * dt;
            ret.theta = curr_veh_state.pos.theta;
        }
        else
        {
            // 圆弧运动
            double r = v / w;
            double delta_theta = w * dt;

            ret.x = curr_veh_state.pos.x + r * (std::sin(curr_veh_state.pos.theta + delta_theta) -
                std::sin(curr_veh_state.pos.theta));
            ret.y = curr_veh_state.pos.y + -r * (std::cos(curr_veh_state.pos.theta + delta_theta) -
                std::cos(curr_veh_state.pos.theta));
            ret.theta = curr_veh_state.pos.theta + delta_theta;
        }

        auto [proj_node, _] = reference_path_->GetProjection({ ret.x, ret.y }, last_veh_proj_nearest_idx_);   // warmup实际用的是本帧的nearest结果
        Path::PointSL sl = Path::Utils::XYtoSL({ ret.x, ret.y }, { proj_node.x, proj_node.y }, proj_node.s, proj_node.theta);
        ret.s = sl.s;
        ret.l = sl.l;

        result_.log << "GetPlanningStart(): use motion extrapolation.\n";
        return ret;
    }

    // 3. 认为控制跟上了，直接从上一帧轨迹中去除对应点作为本帧规划起点
    // todo 上帧数据没有
    result_.log << "GetPlanningStart(): use last planning result.\n";
    return curr_veh_state.pos;
}

std::vector<Path::PathNode> LocalPlanner::SampleReferencePoints(const Path::PathNode & planning_start_point)
{
    double begin_s = std::max(0.0, planning_start_point.s);
    double end_s = std::min(reference_path_->GetLength(), planning_start_point.s + params_.reference_path.TRUNCATED_FORWARD_S);

    std::vector<Path::PathNode> points;
    int s_num = std::ceil((end_s - begin_s) / params_.reference_path.S_INTERVAL);
    double s_step = (end_s - begin_s) / s_num;
    for (int i = 0; i <= s_num; ++i)
    {
        double s = begin_s + i * s_step;
        auto point = reference_path_->GetPathNode(s);
        points.push_back(point);
    }
    curr_s_interval_ = s_step;
    result_.log << "SampleReferencePoints(): sample points num: " << points.size() << ", s_step: " << s_step << "\n";
    return points;
}

Path::PathNode LocalPlanner::GetApproxNode(const Path::PathNode & original_node, 
    const Path::PathNode & actual_node, double len) const
{
    // Point on refrence.
    auto prj_node = reference_path_->GetPathNode(original_node.s + len);
    double x = prj_node.x;
    double y = prj_node.y;
    
    Path::PointXY v1, v2;
    v1.x = actual_node.x - original_node.x;
    v1.y = actual_node.y - original_node.y;
    v2.x = x - original_node.x;
    v2.y = y - original_node.y;
    double proj = (v1.x * v2.x + v1.y * v2.y) / std::max(0.001, std::sqrt(v1.x * v1.x + v1.y * v1.y));
    double move_dis = std::fabs(len) - proj;

    // Move.
    Path::PathNode ret;
    int sign = len >= 0 ? 1 : -1;
    ret.x = x + sign * move_dis * std::cos(original_node.theta);
    ret.y = y + sign * move_dis * std::sin(original_node.theta);
    ret.theta = original_node.theta;
    return ret;
}

std::vector<std::array<std::pair<double, double>, 3>> LocalPlanner::GetBoundsByMap(
    const std::vector<Path::PathNode> & ref_points)
{
    // 在全局规划中，实在路径dp过程中开辟了凸空间，然后再这个凸空间中进行qp的
    // 因此，在局部规划中基于map探索上下边界时，一定已经在凸空间中，所以可以直接进行探索，不必考虑中间有障碍物的情况。
    // 也即，对于任意一个参考点，其l=0处一定在凸空间内，无障碍碰撞，且lb<0, ub>0。因此此处探索边界从0.0开始

    std::vector<std::array<std::pair<double, double>, 3>> bounds;
    bounds.reserve(ref_points.size());

    int idx = 0;
    // 对于整个局部路径的采样点进行遍历
    for (auto && point : ref_points)
    {
        // 每个采样点有三个碰撞圆
        Path::PathNode c0;
        c0.x = point.x + params_.vehicle.CENTER_TO_COLLISION_CENTER * std::cos(point.theta);
        c0.y = point.y + params_.vehicle.CENTER_TO_COLLISION_CENTER * std::sin(point.theta);
        c0.theta = point.theta;

        Path::PathNode c1;
        c1.x = point.x;
        c1.y = point.y;
        c1.theta = point.theta;

        Path::PathNode c2;
        c2.x = point.x - params_.vehicle.CENTER_TO_COLLISION_CENTER * std::cos(point.theta);
        c2.y = point.y - params_.vehicle.CENTER_TO_COLLISION_CENTER * std::sin(point.theta);
        c2.theta = point.theta;

        // 每个采样点构成一个三碰撞圆组
        Vehicle::CollisionCircle cc(*map_, { c0, c1, c2 },
            params_.vehicle.COLLISION_CIRCLE_RADIUS,
            params_.vehicle.COLLISION_SAFETY_MARGIN);

        // 得到该三碰撞圆组的上下边界
        std::array<Path::PointXY, 3> lb_xy;
        std::array<Path::PointXY, 3> ub_xy;
        auto bd = cc.GetCollisionBounds(
            params_.map.BOUND_SEARCH_RANGE,
            params_.map.BOUND_SEARCH_LARGE_RESOLUTION,
            params_.map.BOUND_SEARCH_SMALL_RESOLUTION,
            &lb_xy, &ub_xy);

        // ===============================================================================
        // 上面计算出的边界是相对于车辆中轴线的（尤其是c0,c2），而非相对于参考线的，因此存在误差，需要校正。
        Path::PathNode c0_new = GetApproxNode(point, c0, params_.vehicle.CENTER_TO_COLLISION_CENTER);
        Path::PathNode c2_new = GetApproxNode(point, c2, -params_.vehicle.CENTER_TO_COLLISION_CENTER);
        double offset_0 = Path::Utils::GlobalToLocal(c0, c0_new).y;
        double offset_2 = Path::Utils::GlobalToLocal(c2, c2_new).y;
        // 校正边界
        bd[0].first += offset_0;  bd[0].second += offset_0;
        bd[2].first += offset_2;  bd[2].second += offset_2;

        // ===============================================================================
        // 上面得到了校正后的边界。根据后面路径QP关于碰撞圆的边界约束条件，这里直接对边界进行偏移计算
        double dis = params_.vehicle.CENTER_TO_COLLISION_CENTER * std::sin(point.theta);
        bd[0].first -= dis;     bd[0].second -= dis;
        bd[2].first += dis;     bd[2].second += dis;

        bounds.push_back(bd);

        // 输出调试信息
        result_.log << "point idx: " << idx++ << "\n"
            << "\tc0: (lb, ub), (lb_x, lb_y), (ub_x, ub_y) : (" << bd[0].first << ", " << bd[0].second << "), (" << lb_xy[0].x << ", " << lb_xy[0].y << "), (" << ub_xy[0].x << ", " << ub_xy[0].y << ")\n"
            << "\tc1: (lb, ub), (lb_x, lb_y), (ub_x, ub_y) : (" << bd[1].first << ", " << bd[1].second << "), (" << lb_xy[1].x << ", " << lb_xy[1].y << "), (" << ub_xy[1].x << ", " << ub_xy[1].y << ")\n"
            << "\tc2: (lb, ub), (lb_x, lb_y), (ub_x, ub_y) : (" << bd[2].first << ", " << bd[2].second << "), (" << lb_xy[2].x << ", " << lb_xy[2].y << "), (" << ub_xy[2].x << ", " << ub_xy[2].y << ")\n";
        result_.path_qp_lb.push_back(lb_xy);
        result_.path_qp_ub.push_back(ub_xy);
    }

    return bounds;
}

bool LocalPlanner::PathQP(const std::vector<Path::PathNode> & ref_points,
    const std::vector<std::array<std::pair<double, double>, 3>> & bounds,
    const Path::PathNode & start_point,
    std::vector<Path::PointXY> & optimized_path)
{
    // 设置权重等参数
    const static std::array<double, 4> lateral_weights   = { params_.path_qp.WEIGHT_L,
                                                             params_.path_qp.WEIGHT_DL,
                                                             params_.path_qp.WEIGHT_DDL,
                                                             params_.path_qp.WEIGHT_DDDL };
    const static std::array<double, 3> end_state_weights = { params_.path_qp.WEIGHT_END_STATE_L,
                                                             params_.path_qp.WEIGHT_END_STATE_DL,
                                                             params_.path_qp.WEIGHT_END_STATE_DDL };
    const static double dl_limit = params_.path_qp.DL_LIMIT;
    const static double vehicle_kappa_max = params_.vehicle.MAX_KAPPA;

    Smoother::PiecewiseJerkSmoother2 smoother(lateral_weights, end_state_weights, dl_limit, vehicle_kappa_max);

    // 起点和终点sl位姿
    const auto & start_point_proj = ref_points.front();
    const Path::PointSLWithDerivatives start_point_sl = Path::Utils::XYtoSL(
        { start_point.x, start_point.y }, start_point.theta, start_point.kappa,
        { start_point_proj.x, start_point_proj.y }, start_point_proj.s,
        start_point_proj.theta, start_point_proj.kappa, start_point_proj.dkappa);
    std::array<double, 3> init_state = { start_point_sl.l, start_point_sl.l_prime, start_point_sl.l_double_prime };      // l, l', l"的参考值
    std::array<double, 3> end_state_ref = { 0.0, 0.0, 0.0 };      // l, l', l"的参考值
    result_.log << "PathQP(): start_point_sl (l, l', l\"): ("
        << start_point_sl.l << ", " << start_point_sl.l_prime << ", " << start_point_sl.l_double_prime << ")\n";

    // 传入求解数据进行求解
    auto start_time = std::chrono::steady_clock::now();
    optimized_path.clear();
    if (!smoother.Solve(curr_s_interval_, ref_points, bounds,
        init_state, end_state_ref, optimized_path))
    {
        return false;
    }
    auto end_time = std::chrono::steady_clock::now();

    result_.log << "PathQP(): cost time: " << std::chrono::duration_cast<std::chrono::milliseconds>(end_time - start_time).count() << " ms.\n";
    return true;
}
