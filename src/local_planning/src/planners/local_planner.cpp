#include "local_planning/planners/local_planner.h"


void LocalPlanner::InitParams(const LocalPlannerParams & params)
{
    params_ = params;
}

bool LocalPlanner::Plan(const Path::ReferencePath::Ptr & reference_path,
    const Vehicle::VehicleState & vehicle_state,
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
    // 包括两部分：①代价地图确定的边界；②决策部分给出的每辆车的边界。两部分叠加得到最终上下边界。
    auto bounds = GetBoundsByMap(reference_path, map);

    // 5. 进行路径规划，QP优化
}


std::pair<double, double> LocalPlanner::GetBackwardAndForwardDistance(
    const Path::ReferencePath::Ptr & reference_path) const
{
    double bwd_s = std::max(0.0, vehicle_state_.pos.s - params_.reference_path.TRUNCATED_BACKWARD_S);
    double fwd_s = std::min(reference_path->GetLength(), vehicle_state_.pos.s + params_.reference_path.TRUNCATED_FORWARD_S);
    return { bwd_s, fwd_s };
}

std::vector<std::pair<double, double>> LocalPlanner::GetBoundsByMap(
    const Path::ReferencePath::Ptr & reference_path,
    const Map::MultiMap & map) const
{
    // 在全局规划中，实在路径dp过程中开辟了凸空间，然后再这个凸空间中进行qp的
    // 因此，在局部规划中探索上下边界时，一定已经在凸空间中，所以可以直接进行探索，不必考虑中间有障碍物的情况。
    // 也即，对于任意一个全局参考点，其l=0处一定在凸空间内，无障碍碰撞，且lb<0, ub>0。因此此处探索边界从0.0开始

    auto points = reference_path->GetPathNodes();
    std::vector<std::pair<double, double>> bounds;
    bounds.reserve(points.size());

    const double collision_distance_in_map = (params_.vehicle.COLLISION_CIRCLE_RADIUS
        + params_.vehicle.COLLISION_SAFETY_MARGIN) / map.resolution;
    for (const auto & point : points)
    {
        // 确定下边界， 粗+细
        double lower_bound = 0.0;
        while (lower_bound > -params_.map.BOUND_SEARCH_RANGE)
        {
            lower_bound -= params_.map.BOUND_SEARCH_LARGR_RESOLUTION;
            Path::PointSL sl(point.s, lower_bound);
            Path::PointXY xy = Path::Utils::SLtoXY(sl, { point.x, point.y }, point.theta);
            xy.x = (xy.x - map.origin_x) / map.resolution;
            xy.y = (xy.y - map.origin_y) / map.resolution;

            if (!map.distance_map.IsInside(xy.x, xy.y) ||                               // 此处点已经不在地图内部
                map.distance_map.GetDistance(xy.x, xy.y) < collision_distance_in_map)   // 或者此处点发生了碰撞，则认为上次结果即为边界
            {
                lower_bound += params_.map.BOUND_SEARCH_LARGR_RESOLUTION;
                break;
            }
        }
        while (lower_bound > -params_.map.BOUND_SEARCH_RANGE)
        {
            lower_bound -= params_.map.BOUND_SEARCH_SMALL_RESOLUTION;
            Path::PointSL sl(point.s, lower_bound);
            Path::PointXY xy = Path::Utils::SLtoXY(sl, { point.x, point.y }, point.theta);
            xy.x = (xy.x - map.origin_x) / map.resolution;
            xy.y = (xy.y - map.origin_y) / map.resolution;

            if (!map.distance_map.IsInside(xy.x, xy.y) ||
                map.distance_map.GetDistance(xy.x, xy.y) < collision_distance_in_map)
            {
                lower_bound += params_.map.BOUND_SEARCH_SMALL_RESOLUTION;
                break;
            }
        }

        // 确定上边界， 粗+细
        double upper_bound = 0.0;
        while (upper_bound < params_.map.BOUND_SEARCH_RANGE)
        {
            upper_bound += params_.map.BOUND_SEARCH_LARGR_RESOLUTION;
            Path::PointSL sl(point.s, upper_bound);
            Path::PointXY xy = Path::Utils::SLtoXY(sl, { point.x, point.y }, point.theta);
            xy.x = (xy.x - map.origin_x) / map.resolution;
            xy.y = (xy.y - map.origin_y) / map.resolution;

            if (!map.distance_map.IsInside(xy.x, xy.y) ||                               // 此处点已经不在地图内部
                map.distance_map.GetDistance(xy.x, xy.y) < collision_distance_in_map)   // 或者此处点发生了碰撞，则认为上次结果即为边界
            {
                upper_bound -= params_.map.BOUND_SEARCH_LARGR_RESOLUTION;
                break;
            }
        }
        while (upper_bound < params_.map.BOUND_SEARCH_RANGE)
        {
            upper_bound += params_.map.BOUND_SEARCH_SMALL_RESOLUTION;
            Path::PointSL sl(point.s, upper_bound);
            Path::PointXY xy = Path::Utils::SLtoXY(sl, { point.x, point.y }, point.theta);
            xy.x = (xy.x - map.origin_x) / map.resolution;
            xy.y = (xy.y - map.origin_y) / map.resolution;

            if (!map.distance_map.IsInside(xy.x, xy.y) ||
                map.distance_map.GetDistance(xy.x, xy.y) < collision_distance_in_map)
            {
                upper_bound -= params_.map.BOUND_SEARCH_SMALL_RESOLUTION;
                break;
            }
        }

        bounds.emplace_back(lower_bound, upper_bound);
    }

    return bounds;
}
