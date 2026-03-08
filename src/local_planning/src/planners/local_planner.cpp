// local_planner.cpp
#include "local_planning/planners/local_planner.h"


void LocalPlanner::InitParams(const LocalPlannerParamsInterface & params)
{
    LocalPlannerParams p = dynamic_cast<const LocalPlannerParams &>(params);
    params_ = p;
    
    // 初始化决策模块
    decision_maker_ = std::make_shared<Decision::DecisionMaker>();
    decision_maker_->Initialize(params_.decision_params);
}

void LocalPlanner::SetMap(const Map::MultiMap::Ptr & map)
{
    map_ = map;
    flag_map_ = true;
    if (decision_maker_)
    {
        decision_maker_->SetCostMap(map_);
    }
}

void LocalPlanner::SetReferencePath(const Path::ReferencePath::Ptr & reference_path)
{
    reference_path_ = reference_path;
    flag_reference_path_ = true;
    if (decision_maker_)
    {
        decision_maker_->SetReferencePath(reference_path_);
    }
}

void LocalPlanner::SetVehicleState(const Vehicle::State::Ptr & vehicle_state)
{
    vehicle_state_ = vehicle_state;
    flag_vehicle_state_ = true;

    auto now = std::chrono::steady_clock::now();
    if (last_planning_time_ == std::chrono::steady_clock::time_point())
    {
        last_planning_time_ = now;
    }
}

void LocalPlanner::SetObstacles(const Obstacle::Obstacle::List & obstacles)
{
    obstacles_ = obstacles;
    flag_obstacles_ = true;
}

bool LocalPlanner::Plan(LocalPlannerResult & result, std::string & error_msg)
{
    result_.Clear();
    error_msg.clear();
    // 记录当前绝对时间
    current_abs_time_ = std::chrono::duration<double>(std::chrono::steady_clock::now().time_since_epoch()).count();

    // 0. 判断当前帧数据是否完整
    if (!IsDataReady())
    {
        error_msg = "LocalPlanner::Plan() failed: data not ready.";
        has_last_trajectory_ = false;
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
                << "curr_veh_proj_nearest_idx: " << curr_veh_proj_nearest_idx << "\n"
                << "curr_veh_proj_point:\n" << curr_veh_proj_point << "\n";

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

    // 4. 执行决策
    result_.log << "Starting decision making...\n";
    decision_maker_->UpdateAndDecide(obstacles_,
        planning_start_point,
        curr_veh_state.v,
        0.0);  // 暂时使用0加速度
    // result_.log << "Decision results:\n";
    // result_.log << decision_maker_->GetDebugInfo();

    // 5. 确定路径规划的上下边界
    // 包括两部分：①代价地图确定的边界；②决策部分给出的每辆车的边界。两部分每个点都是三碰撞圆的边界，两部分求交集得到最终上下边界。
    result_.log << "Calculating map boundaries...\n";
    auto map_bounds = GetBoundsByMap(ref_points);
    
    std::vector<std::array<std::pair<double, double>, 3>> final_bounds;
    if (flag_obstacles_ && !obstacles_.empty())
    {
        result_.log << "Generating decision boundaries...\n";
        auto decision_boundary = decision_maker_->GeneratePathBoundary(ref_points);
        final_bounds = MergeBounds(map_bounds, decision_boundary);
        result_.log << "Merged " << map_bounds.size() << " map bounds with " 
                    << decision_boundary.bounds.size() << " decision bounds.\n";
    }
    else
    {
        final_bounds = map_bounds;
        result_.log << "No decision boundaries, using map bounds only.\n";
    }

    // 6. 路径规划
    std::vector<Path::PathNode> optimized_path;
    if (!PathPlanning(ref_points, final_bounds, planning_start_point, optimized_path))
    {
        error_msg = "LocalPlanner::Plan() failed: PathPlanning failed.";
        has_last_trajectory_ = false;
        return false;
    }
    result_.log << "Path planning generated " << optimized_path.size() << " path nodes.\n";

    // 7. 速度规划
    std::vector<Path::TrajectoryPoint> speed_profile;
    if (!SpeedPlanning(curr_veh_state, speed_profile))
    {
        error_msg = "LocalPlanner::Plan() failed: SpeedPlanning failed.";
        has_last_trajectory_ = false;
        return false;
    }
    result_.log << "Speed planning generated " << speed_profile.size() << " speed points.\n";

    // 8. 生成最终轨迹
    GenerateTrajectory(speed_profile, optimized_path, result_.trajectory);
    result_.log << "Generated " << result_.trajectory.size() << " trajectory points.\n";
    // 将本帧轨迹时间转换为绝对时间（用于存储和下一帧使用）
    for (auto & pt : result_.trajectory)
    {
        pt.t += current_abs_time_;
    }

    // 99. 记录结果
    last_start_point_ = planning_start_point;
    last_trajectory_ = result_.trajectory;  // 此时已是绝对时间
    has_last_trajectory_ = true;

    // 记录规划时间
    auto current_time = std::chrono::steady_clock::now();
    last_planning_cycle_time_ = std::chrono::duration<double>(current_time - last_planning_time_).count();
    last_planning_time_ = current_time;

    
    result_.planning_cost_time = std::chrono::duration<double>(current_time.time_since_epoch()).count() - result_.timestamp;
    result_.log << "LocalPlanner::Plan() end.\n"
                << "LocalPlanner::Plan() elapsed time: " << result_.planning_cost_time * 1000.0 << " ms.\n";
    
    result = std::move(result_);
    return true;
}

bool LocalPlanner::IsDataReady() const
{
    return flag_map_ && flag_reference_path_ && flag_vehicle_state_;
}

bool LocalPlanner::MatchLastTrajectory(const Path::PathNode & current_pos,
    Path::TrajectoryPoint & matched_point,
    double & lateral_error,
    double & longitudinal_error)
{
    if (last_trajectory_.empty()) return false;

    double min_dis_sq = std::numeric_limits<double>::max();
    size_t min_index = 0;
    for (size_t i = 0; i < last_trajectory_.size(); ++i)
    {
        const auto & point = last_trajectory_[i];
        double dx = point.x - current_pos.x;
        double dy = point.y - current_pos.y;
        double dis_sq = dx * dx + dy * dy;
        if (dis_sq < min_dis_sq)
        {
            min_dis_sq = dis_sq;
            min_index = i;
        }
    }
    matched_point = last_trajectory_[min_index];

    double cos_theta = std::cos(matched_point.theta);
    double sin_theta = std::sin(matched_point.theta);
    double dx = current_pos.x - matched_point.x;
    double dy = current_pos.y - matched_point.y;
    longitudinal_error = dx * cos_theta + dy * sin_theta;
    lateral_error = -dx * sin_theta + dy * cos_theta;

    double total_error = std::sqrt(lateral_error * lateral_error + longitudinal_error * longitudinal_error);
    result_.log << "MatchLastTrajectory(): matched point index: " << min_index
                << ", total error: " << total_error << " m\n";
    return true;
}

Path::PathNode LocalPlanner::GetMotionExtrapolationStart(const Vehicle::State & curr_veh_state)
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
    ret.kappa = curr_veh_state.pos.kappa;

    // 获取在参考线上的投影
    auto [proj_node, proj_idx] = reference_path_->GetProjection(
        { ret.x, ret.y }, last_veh_proj_nearest_idx_);

    Path::PointSL sl = Path::Utils::XYtoSL({ ret.x, ret.y }, { proj_node.x, proj_node.y },
        proj_node.s, proj_node.theta);

    ret.s = sl.s;
    ret.l = sl.l;
    return ret;
}

Path::PathNode LocalPlanner::GetPlanningStart(const Vehicle::State & curr_veh_state, const Path::PathNode & veh_proj_point)
{
    // 1. 车速为0，直接认为当前车辆位置就是规划起点
    if (curr_veh_state.v < params_.start_point.V_TOLERANCE &&
        curr_veh_state.w < params_.start_point.W_TOLERANCE)
    {
        result_.log << "GetPlanningStart(): vehicle is stopped, using current position as planning start.\n";
        return curr_veh_state.pos;
    }

    // 2. 检查是否有上一帧轨迹可用
    if (!has_last_trajectory_ || last_trajectory_.empty())
    {
        // 没有上一帧轨迹，使用运动学外推
        result_.log << "GetPlanningStart(): no last trajectory, using motion extrapolation.\n";
        return GetMotionExtrapolationStart(curr_veh_state);
    }

    // 3. 检查当前位置与上一帧轨迹的匹配度
    Path::TrajectoryPoint matched_point;
    double lateral_error, longitudinal_error;
    if (MatchLastTrajectory(curr_veh_state.pos, matched_point, lateral_error, longitudinal_error))
    {
        // 判断是否控制跟上（误差在容忍范围内）
        double pos_error = std::sqrt(lateral_error * lateral_error + longitudinal_error * longitudinal_error);
        if (pos_error <= params_.start_point.POS_TOLERANCE)
        {
            // 控制跟上了，使用上一帧轨迹中的匹配点作为规划起点
            result_.log << "GetPlanningStart(): control is following, using matched point from last trajectory.\n";
            result_.log << "  Position error: " << pos_error << " m (lateral: " << lateral_error
                        << " m, longitudinal: " << longitudinal_error << " m)\n";

            // 将匹配点转换为PathNode
            Path::PathNode planning_start;
            planning_start.x = matched_point.x;
            planning_start.y = matched_point.y;
            planning_start.theta = matched_point.theta;
            planning_start.kappa = matched_point.kappa;

            // 计算在参考线上的投影获取s和l
            auto [proj_node, proj_idx] = reference_path_->GetProjection(
                { planning_start.x, planning_start.y }, last_veh_proj_nearest_idx_);

            Path::PointSL sl = Path::Utils::XYtoSL(
                { planning_start.x, planning_start.y },
                { proj_node.x, proj_node.y },
                proj_node.s,
                proj_node.theta);

            planning_start.s = sl.s;
            planning_start.l = sl.l;
            return planning_start;
        }
        else
        {
            result_.log << "GetPlanningStart(): control is NOT following, using motion extrapolation.\n";
            result_.log << "  Position error: " << pos_error << " m (exceeds tolerance: "
                        << params_.start_point.POS_TOLERANCE << " m)\n";
            return GetMotionExtrapolationStart(curr_veh_state);
        }
    }
    else
    {
        result_.log << "GetPlanningStart(): cannot match last trajectory, using motion extrapolation.\n";
        return GetMotionExtrapolationStart(curr_veh_state);
    }
}

std::vector<Path::PathNode> LocalPlanner::SampleReferencePoints(const Path::PathNode & planning_start_point)
{
    double begin_s = std::max(0.0, planning_start_point.s);
    double end_s = std::min(reference_path_->GetLength(),
                             planning_start_point.s + params_.reference_path.TRUNCATED_FORWARD_S);
    int s_num = std::ceil((end_s - begin_s) / params_.reference_path.S_INTERVAL);
    double s_step = (end_s - begin_s) / s_num;
    std::vector<Path::PathNode> points;
    for (int i = 0; i <= s_num; ++i)
    {
        double s = begin_s + i * s_step;
        points.push_back(reference_path_->GetPathNode(s));
    }
    curr_s_interval_ = s_step;
    result_.log << "SampleReferencePoints(): sample points num: " << points.size() << ", s_step: " << s_step << "\n";
    return points;
}

Path::PathNode LocalPlanner::GetApproxNode(const Path::PathNode & original_node,
    const Path::PathNode & actual_node, double len) const
{
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
        // result_.log << "point idx: " << idx++ << "\n"
        //     << "\tc0: (lb, ub), (lb_x, lb_y), (ub_x, ub_y) : (" << bd[0].first << ", " << bd[0].second << "), (" << lb_xy[0].x << ", " << lb_xy[0].y << "), (" << ub_xy[0].x << ", " << ub_xy[0].y << ")\n"
        //     << "\tc1: (lb, ub), (lb_x, lb_y), (ub_x, ub_y) : (" << bd[1].first << ", " << bd[1].second << "), (" << lb_xy[1].x << ", " << lb_xy[1].y << "), (" << ub_xy[1].x << ", " << ub_xy[1].y << ")\n"
        //     << "\tc2: (lb, ub), (lb_x, lb_y), (ub_x, ub_y) : (" << bd[2].first << ", " << bd[2].second << "), (" << lb_xy[2].x << ", " << lb_xy[2].y << "), (" << ub_xy[2].x << ", " << ub_xy[2].y << ")\n";
        result_.path_qp_lb.push_back(lb_xy);
        result_.path_qp_ub.push_back(ub_xy);
    }

    return bounds;
}

std::vector<std::array<std::pair<double, double>, 3>> LocalPlanner::MergeBounds(
    const std::vector<std::array<std::pair<double, double>, 3>> & map_bounds,
    const Decision::PathBoundary & decision_bounds)
{
    std::vector<std::array<std::pair<double, double>, 3>> merged_bounds;
    
    // 确保地图边界和决策边界的大小一致
    size_t num_points = std::min(map_bounds.size(), decision_bounds.bounds.size());
    merged_bounds.reserve(num_points);

    for (size_t i = 0; i < num_points; ++i)
    {
        std::array<std::pair<double, double>, 3> merged_point_bounds;
        
        for (int j = 0; j < 3; ++j)  // 遍历三个碰撞圆
        {
            // 取交集：下界取较大值，上界取较小值
            double lower = std::max(map_bounds[i][j].first, decision_bounds.bounds[i][j].first);
            double upper = std::min(map_bounds[i][j].second, decision_bounds.bounds[i][j].second);
            // 确保边界有效性
            if (lower > upper)
            {
                // 如果边界冲突，进行适当调整（取中间值）
                double mid = (lower + upper) / 2.0;
                lower = mid - 0.1;
                upper = mid + 0.1;
                result_.log << "Warning: Bound conflict at point " << i 
                            << ", circle " << j << ". Adjusted bounds to [" 
                            << lower << ", " << upper << "]\n";
            }
            merged_point_bounds[j] = std::make_pair(lower, upper);
        }
        merged_bounds.push_back(merged_point_bounds);
    }
    // 补足剩余点
    for (size_t i = num_points; i < map_bounds.size(); ++i)
    {
        merged_bounds.push_back(map_bounds[i]);
    }
    result_.log << "Merged bounds: " << merged_bounds.size() << " points.\n";
    return merged_bounds;
}

bool LocalPlanner::PathPlanning(const std::vector<Path::PathNode> & ref_points,
                                const std::vector<std::array<std::pair<double, double>, 3>> & bounds,
                                const Path::PathNode & start_point,
                                std::vector<Path::PathNode> & optimized_path)
{
    // 设置权重等参数
    const static Smoother::PiecewiseJerkPathSmoother2::Weights weights {
        params_.path_qp.WEIGHT_L,
        params_.path_qp.WEIGHT_DL,
        params_.path_qp.WEIGHT_DDL,
        params_.path_qp.WEIGHT_DDDL,
        params_.path_qp.WEIGHT_END_STATE_L,
        params_.path_qp.WEIGHT_END_STATE_DL,
        params_.path_qp.WEIGHT_END_STATE_DDL
    };
    const static Smoother::PiecewiseJerkPathSmoother2::Params params {
        params_.path_qp.DL_LIMIT,
        params_.vehicle.MAX_KAPPA
    };

    // 优化器
    const static Smoother::PiecewiseJerkPathSmoother2 smoother(weights, params);

    // 起点和终点sl位姿
    const auto & start_point_proj = ref_points.front();
    const Path::PointSLWithDerivatives start_point_sl = Path::Utils::XYtoSL(
        { start_point.x, start_point.y }, start_point.theta, start_point.kappa,
        { start_point_proj.x, start_point_proj.y }, start_point_proj.s,
        start_point_proj.theta, start_point_proj.kappa, start_point_proj.dkappa);
    std::array<double, 3> init_state = { start_point_sl.l, start_point_sl.l_prime, start_point_sl.l_double_prime };
    std::array<double, 3> end_state_ref = { 0.0, 0.0, 0.0 };
    result_.log << "PathPlanning(): start_point_sl (l, l', l\"): ("
                << start_point_sl.l << ", " << start_point_sl.l_prime << ", " << start_point_sl.l_double_prime << ")\n";

    auto start_time = std::chrono::steady_clock::now();

    // 调用优化器，得到路径点的 SL 信息（包含 l, l', l"）
    std::vector<Path::PointSLWithDerivatives> path_sl;
    if (!smoother.Solve(curr_s_interval_, ref_points, bounds,
                        init_state, end_state_ref, path_sl))
        return false;

    optimized_path.clear();
    optimized_path.reserve(path_sl.size());

    for (size_t i = 0; i < path_sl.size(); ++i)
    {
        const auto & sl = path_sl[i];
        const auto & ref_node = ref_points[i];

        Path::PathNode node;
        node.s = sl.s;
        node.l = sl.l;
        node.dkappa = sl.l_prime;  // 存储 l' 到 dkappa 字段

        // 计算笛卡尔坐标
        node.x = ref_node.x - sl.l * std::sin(ref_node.theta);
        node.y = ref_node.y + sl.l * std::cos(ref_node.theta);

        // 计算航向角 theta
        double denominator = 1.0 - ref_node.kappa * sl.l;
        if (std::abs(denominator) < 1e-6)
            denominator = 1e-6;
        node.theta = ref_node.theta + std::atan2(sl.l_prime, denominator);

        // 计算曲率 kappa
        double numerator = (ref_node.kappa + sl.l_double_prime) * (1.0 - ref_node.kappa * sl.l);
        denominator = std::pow((1.0 - ref_node.kappa * sl.l) * (1.0 - ref_node.kappa * sl.l) +
                                sl.l_prime * sl.l_prime, 1.5);
        node.kappa = numerator / (denominator + 1e-9);  // 避免除零

        optimized_path.push_back(node);
    }

    auto end_time = std::chrono::steady_clock::now();
    result_.log << "PathPlanning(): cost time: " << std::chrono::duration_cast<std::chrono::milliseconds>(end_time - start_time).count() << " ms.\n";
    return true;
}

double LocalPlanner::EstimateSFromLastTrajectory(const Vehicle::State & curr_veh_state, double abs_t) const
{
    if (!has_last_trajectory_ || last_trajectory_.empty())
        return curr_veh_state.pos.s;

    const auto & traj = last_trajectory_;
    auto it = std::lower_bound(traj.begin(), traj.end(), abs_t,
        [](const Path::TrajectoryPoint & pt, double val) { return pt.t < val; });
    if (it == traj.begin())
        return traj.front().s;
    if (it == traj.end())
        return traj.back().s;
    auto prev = std::prev(it);
    double t0 = prev->t, t1 = it->t;
    double s0 = prev->s, s1 = it->s;
    double ratio = (abs_t - t0) / (t1 - t0 + 1e-9);
    return s0 + ratio * (s1 - s0);
}

double LocalPlanner::GetCurvatureFromLastTrajectory(double s) const
{
    if (!has_last_trajectory_ || last_trajectory_.empty())
        return 0.0;
    const auto & traj = last_trajectory_;
    auto it = std::lower_bound(traj.begin(), traj.end(), s,
        [](const Path::TrajectoryPoint & pt, double val) { return pt.s < val; });
    if (it == traj.begin())
        return traj.front().kappa;
    if (it == traj.end())
        return traj.back().kappa;
    auto prev = std::prev(it);
    double s0 = prev->s, s1 = it->s;
    double k0 = prev->kappa, k1 = it->kappa;
    double ratio = (s - s0) / (s1 - s0 + 1e-9);
    return k0 + ratio * (k1 - k0);
}

std::vector<std::pair<double, double>> LocalPlanner::GenerateVelocityBoundary(
    const Vehicle::State & curr_veh_state, const std::vector<double> & time_points) const
{
    size_t N = time_points.size();
    std::vector<std::pair<double, double>> v_bounds(N,
        std::make_pair(params_.vehicle.MIN_SPEED, params_.vehicle.MAX_SPEED));

    if (!has_last_trajectory_ || last_trajectory_.empty())
    {
        // 退化：使用参考线曲率
        for (size_t i = 0; i < N; ++i)
        {
            double s_est = curr_veh_state.pos.s + curr_veh_state.v * time_points[i];
            double kappa = 0.0;
            if (s_est >= 0 && s_est <= reference_path_->GetLength())
                kappa = reference_path_->GetPathNode(s_est).kappa;
            double v_curv = std::sqrt(params_.vehicle.MAX_LATERAL_ACCEL / (std::fabs(kappa) + 1e-6));
            v_bounds[i].second = std::min(params_.vehicle.MAX_SPEED, v_curv);
        }
        return v_bounds;
    }

    for (size_t i = 0; i < N; ++i)
    {
        double abs_t = current_abs_time_ + time_points[i];  // 绝对时间
        double s_est = EstimateSFromLastTrajectory(curr_veh_state, abs_t);
        double kappa = GetCurvatureFromLastTrajectory(s_est);
        double v_curv = std::sqrt(params_.vehicle.MAX_LATERAL_ACCEL / (std::fabs(kappa) + 1e-6));
        v_bounds[i].second = std::min(params_.vehicle.MAX_SPEED, v_curv);
    }
    return v_bounds;
}

bool LocalPlanner::SpeedPlanning(const Vehicle::State & curr_veh_state, std::vector<Path::TrajectoryPoint> & optimized_speed_profile)
{
    // 1. 获取决策生成的 s 边界
    auto speed_boundary = decision_maker_->GenerateSpeedBoundary(
        params_.speed_qp.PLANNING_TIME_HORIZON,
        params_.speed_qp.TIME_RESOLUTION);
    if (speed_boundary.time_points.empty())
    {
        result_.log << "SpeedPlanning failed: no speed boundary generated.\n";
        return false;
    }

    const auto & time_points = speed_boundary.time_points;
    const auto & s_bounds = speed_boundary.bounds;
    size_t N = time_points.size();

    // 2. 提取 s 下界和上界
    std::vector<double> s_lower(N), s_upper(N);
    for (size_t i = 0; i < N; ++i)
    {
        s_lower[i] = s_bounds[i].first;
        s_upper[i] = s_bounds[i].second;
    }

    // 3. 基于曲率生成速度硬约束
    auto v_bounds_curv = GenerateVelocityBoundary(curr_veh_state, time_points);

    // 4. 基于决策生成速度硬约束
    auto v_bounds_decision = decision_maker_->GenerateVConstraint(time_points);

    // 5. 融合两个速度约束（取交集）
    std::vector<std::pair<double, double>> v_bounds_final(N);
    for (size_t i = 0; i < N; ++i)
    {
        double lower = std::max(v_bounds_curv[i].first, v_bounds_decision[i].first);
        double upper = std::min(v_bounds_curv[i].second, v_bounds_decision[i].second);
        if (lower > upper)
        {
            double mid = (lower + upper) / 2.0;
            lower = mid;
            upper = mid;
        }
        v_bounds_final[i] = std::make_pair(lower, upper);
    }

    // 6. 分离 v 下界和上界
    std::vector<double> v_lower(N), v_upper(N);
    for (size_t i = 0; i < N; ++i)
    {
        v_lower[i] = v_bounds_final[i].first;
        v_upper[i] = v_bounds_final[i].second;
    }

    // 7. 加速度边界
    std::vector<double> a_lower(N, params_.vehicle.MAX_DECELERATION);
    std::vector<double> a_upper(N, params_.vehicle.MAX_ACCELERATION);

    // 8. 参考速度（软约束）
    std::vector<double> v_ref(N, params_.speed_qp.V_REF);
    std::vector<double> kappa_ref(N, 0.0);

    // 9. 权重（静态）
    const static Smoother::PiecewiseJerkSpeedSmoother::Weights weights {
        params_.speed_qp.WEIGHT_SPEED_DEVIATION,
        params_.speed_qp.WEIGHT_ACCELERATION,
        params_.speed_qp.WEIGHT_JERK,
        params_.speed_qp.WEIGHT_LATERAL_ACCELERATION
    };

    // 10. 优化器
    const static Smoother::PiecewiseJerkSpeedSmoother optimizer(params_.speed_qp.TIME_RESOLUTION, weights);

    // 11. 初始状态
    double init_s = std::min(curr_veh_state.pos.s, s_lower.front());
    double init_v = curr_veh_state.v;
    double init_a = 0.0;

    auto start_time = std::chrono::steady_clock::now();
    optimized_speed_profile.clear();
    if (!optimizer.Solve(init_s, init_v, init_a,
                         s_lower, s_upper,
                         v_lower, v_upper,
                         a_lower, a_upper,
                         v_ref, kappa_ref,
                         reference_path_->GetLength(),
                         optimized_speed_profile))
    {
        result_.log << "SpeedPlanning: optimizer failed.\n";
        return false;
    }

    // 12. 设置时间点
    if (optimized_speed_profile.size() != N)
    {
        result_.log << "SpeedPlanning: optimizer returned different number of points.\n";
        return false;
    }
    for (size_t i = 0; i < N; ++i)
    {
        optimized_speed_profile[i].t = time_points[i];
    }

    auto end_time = std::chrono::steady_clock::now();
    result_.log << "SpeedPlanning(): cost time: "
                << std::chrono::duration_cast<std::chrono::milliseconds>(end_time - start_time).count()
                << " ms.\n";
    result_.log << "SpeedPlanning: generated " << optimized_speed_profile.size()
                << " speed points, final speed: " << optimized_speed_profile.back().v
                << " m/s, final s: " << optimized_speed_profile.back().s << " m.\n";
    return true;
}

void LocalPlanner::GenerateTrajectory(const std::vector<Path::TrajectoryPoint> & speed_profile,
                                      const std::vector<Path::PathNode> & optimized_path,
                                      std::vector<Path::TrajectoryPoint> & trajectory)
{
    trajectory.clear();
    if (speed_profile.empty() || optimized_path.empty())
    {
        result_.log << "GenerateTrajectory: input data is empty.\n";
        return;
    }

    // 计算路径点的累积弧长 s_coordinates（实际行驶距离）
    std::vector<double> s_coordinates(optimized_path.size(), optimized_path.front().s);
    for (size_t i = 1; i < optimized_path.size(); ++i)
    {
        double dx = optimized_path[i].x - optimized_path[i-1].x;
        double dy = optimized_path[i].y - optimized_path[i-1].y;
        s_coordinates[i] = s_coordinates[i-1] + std::sqrt(dx*dx + dy*dy);
    }
    result_.log << "Computed path length: " << s_coordinates.back() << " m, " << s_coordinates.size() << " points.\n";

    // 提取路径点的参考线 s、l、l' 和 kappa
    std::vector<double> path_s_ref(optimized_path.size());
    std::vector<double> path_l(optimized_path.size());
    std::vector<double> path_l_prime(optimized_path.size());
    std::vector<double> path_kappa(optimized_path.size());
    for (size_t i = 0; i < optimized_path.size(); ++i)
    {
        path_s_ref[i]   = optimized_path[i].s;
        path_l[i]       = optimized_path[i].l;
        path_l_prime[i] = optimized_path[i].dkappa;
        path_kappa[i]   = optimized_path[i].kappa;
    }

    // 对每个速度点进行插值
    for (const auto & sp : speed_profile)
    {
        double target_s = sp.s;   // 速度点对应的实际弧长

        // 在 s_coordinates 中二分查找
        auto it = std::lower_bound(s_coordinates.begin(), s_coordinates.end(), target_s);
        size_t idx = it - s_coordinates.begin();
        size_t idx0, idx1;
        if (idx == 0)
        {
            idx0 = idx1 = 0;
        }
        else if (idx >= s_coordinates.size())
        {
            idx0 = idx1 = s_coordinates.size() - 1;
        }
        else
        {
            idx0 = idx - 1;
            idx1 = idx;
        }

        double s0 = s_coordinates[idx0];
        double s1 = s_coordinates[idx1];
        double ratio = (target_s - s0) / (s1 - s0 + 1e-9);

        // 插值得到参考线 s、l、l'、kappa
        double ref_s   = path_s_ref[idx0]   + ratio * (path_s_ref[idx1] - path_s_ref[idx0]);
        double l       = path_l[idx0]       + ratio * (path_l[idx1] - path_l[idx0]);
        double l_prime = path_l_prime[idx0] + ratio * (path_l_prime[idx1] - path_l_prime[idx0]);
        double kappa   = path_kappa[idx0]   + ratio * (path_kappa[idx1] - path_kappa[idx0]);

        // 获取参考线上对应 ref_s 的点
        Path::PathNode ref_node = reference_path_->GetPathNode(ref_s);

        // 计算笛卡尔坐标和航向角（航向角也可用插值，但此处仍用公式）
        Path::TrajectoryPoint traj_point;
        traj_point.t = sp.t;
        traj_point.s = sp.s;
        traj_point.v = sp.v;
        traj_point.a = sp.a;
        traj_point.j = sp.j;
        traj_point.l = l;

        traj_point.x = ref_node.x - l * std::sin(ref_node.theta);
        traj_point.y = ref_node.y + l * std::cos(ref_node.theta);

        double denom = 1.0 - ref_node.kappa * l;
        if (std::abs(denom) < 1e-6) denom = 1e-6;
        traj_point.theta = ref_node.theta + std::atan2(l_prime, denom);

        // 直接使用插值得到的曲率
        traj_point.kappa = kappa;

        trajectory.push_back(traj_point);
    }

    result_.log << "GenerateTrajectory: generated " << trajectory.size()
                << " trajectory points, last s: " << trajectory.back().s
                << " m, last t: " << trajectory.back().t << " s.\n";
}