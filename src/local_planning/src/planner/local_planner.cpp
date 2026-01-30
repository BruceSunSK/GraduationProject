#include "local_planning/planner/local_planner.h"


void LocalPlanner::InitParams(const LocalPlannerParams & params)
{
    params_ = params;
}

bool LocalPlanner::Plan(const Path::ReferencePath::Ptr & reference_path,
    const Vehicle::VehicleState & vehicle_state)
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
    const auto bwd_fwd_s = BackwardAndForwardDistance(reference_path);
    Path::ReferencePath::Ptr truncated_reference_path =
        reference_path->GetSegment(bwd_fwd_s.first, bwd_fwd_s.second, params_.reference_path.S_INTERVAL);

}


std::pair<double, double> LocalPlanner::BackwardAndForwardDistance(
    const Path::ReferencePath::Ptr & reference_path) const
{
    double bwd_s = std::max(0.0, vehicle_state_.pos.s - params_.reference_path.TRUNCATED_BACKWARD_S);
    double fwd_s = std::min(reference_path->GetLength(), vehicle_state_.pos.s + params_.reference_path.TRUNCATED_FORWARD_S);
    return { bwd_s, fwd_s };
}