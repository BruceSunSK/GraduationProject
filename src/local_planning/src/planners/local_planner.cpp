#include "local_planning/planners/local_planner.h"


void LocalPlanner::InitParams(const LocalPlannerParams & params)
{
    params_ = params;
}

bool LocalPlanner::Plan(const Path::ReferencePath::Ptr & reference_path,
    const Vehicle::State & vehicle_state,
    const Map::MultiMap & map)
{
    // 1. 寻找车辆在参考线上的投影点 并 更新车辆信息
    static int last_veh_proj_nearest_idx = 0;
    auto [veh_proj_point, veh_proj_nearest_idx] = reference_path->GetProjection(
        { vehicle_state.pos.x, vehicle_state.pos.y }, last_veh_proj_nearest_idx);
    last_veh_proj_nearest_idx = veh_proj_nearest_idx;

    Path::PointXY veh_xy = { vehicle_state.pos.x, vehicle_state.pos.y };
    Path::PointSL veh_sl = Path::Utils::XYtoSL(veh_xy,
        { veh_proj_point.x, veh_proj_point.y }, veh_proj_point.s, veh_proj_point.theta);
    vehicle_state_ = vehicle_state;
    vehicle_state_.pos.s = veh_sl.s;
    vehicle_state_.pos.l = veh_sl.l;

    // 2. 截断参考线
    const auto bwd_fwd_s = GetBackwardAndForwardDistance(reference_path);
    Path::ReferencePath::Ptr truncated_reference_path =
        reference_path->GetSegment(bwd_fwd_s.first, bwd_fwd_s.second, params_.reference_path.S_INTERVAL);

    // 3. 决策
    // todo

    // 4. 确定路径规划的上下边界
    // 包括两部分：①代价地图确定的边界；②决策部分给出的每辆车的边界。两部分每个点都是三碰撞圆的边界，两部分求交集得到最终上下边界。
    auto map_bounds = GetBoundsByMap(truncated_reference_path, map);

    // 5. 进行路径规划，QP优化
}


std::pair<double, double> LocalPlanner::GetBackwardAndForwardDistance(
    const Path::ReferencePath::Ptr & reference_path) const
{
    double bwd_s = std::max(0.0, vehicle_state_.pos.s - params_.reference_path.TRUNCATED_BACKWARD_S);
    double fwd_s = std::min(reference_path->GetLength(), vehicle_state_.pos.s + params_.reference_path.TRUNCATED_FORWARD_S);
    return { bwd_s, fwd_s };
}

Path::PathNode LocalPlanner::GetApproxNode(const Path::ReferencePath::Ptr & reference_path,
    const Path::PathNode & original_node, const Path::PathNode & actual_node, double len) const
{
    // Point on refrence.
    auto prj_node = reference_path->GetPathNode(original_node.s + len);
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
    const Path::ReferencePath::Ptr & reference_path,
    const Map::MultiMap & map) const
{
    // 在全局规划中，实在路径dp过程中开辟了凸空间，然后再这个凸空间中进行qp的
    // 因此，在局部规划中基于map探索上下边界时，一定已经在凸空间中，所以可以直接进行探索，不必考虑中间有障碍物的情况。
    // 也即，对于任意一个参考点，其l=0处一定在凸空间内，无障碍碰撞，且lb<0, ub>0。因此此处探索边界从0.0开始

    auto points = reference_path->GetPathNodes();
    std::vector<std::array<std::pair<double, double>, 3>> bounds;
    bounds.reserve(points.size());

    // 对于整个局部路径的采样点进行遍历
    for (auto && point : points)
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
        Vehicle::CollisionCircle cc(map, { c0, c1, c2 },
            params_.vehicle.COLLISION_CIRCLE_RADIUS,
            params_.vehicle.COLLISION_SAFETY_MARGIN);

        // 得到该三碰撞圆组的上下边界
        auto bd = cc.GetCollisionBounds(
            params_.map.BOUND_SEARCH_RANGE,
            params_.map.BOUND_SEARCH_LARGE_RESOLUTION,
            params_.map.BOUND_SEARCH_SMALL_RESOLUTION);

        // ===============================================================================
        // 上面的判断方法有较大误差，因此需要对对碰撞圆圆心进行校正
        Path::PathNode c0_new = GetApproxNode(reference_path, point, c0, params_.vehicle.CENTER_TO_COLLISION_CENTER);
        Path::PathNode c2_new = GetApproxNode(reference_path, point, c2, -params_.vehicle.CENTER_TO_COLLISION_CENTER);
        // 获得校正边界偏移量
        double offset_0 = Path::Utils::GlobalToLocal(c0, c0_new).y;
        double offset_2 = Path::Utils::GlobalToLocal(c2, c2_new).y;
        // 校正边界
        bd[0].first += offset_0;  bd[0].second += offset_0;
        bd[2].first += offset_2;  bd[2].second += offset_2;

        bounds.push_back(bd);
    }

    return bounds;
}
