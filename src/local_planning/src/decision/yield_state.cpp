// yield_state.cpp
#include "local_planning/decision/yield_state.h"


namespace Decision
{
DecisionType YieldState::Evaluate(DecisionContext & context)
{
    double distance = context.projection.distance;      // 障碍物投影纵向碰撞距离，已经考虑了自车、他车的长度
    double ttc = context.projection.time_to_collision;

    // 1. 需要停车？
    if (ShouldStop(context))
        return DecisionType::STOP;

    // 2. 可以安全通过？
    if (IsClearToPass(context))
    {
        if (distance > 40.0)
            return DecisionType::FOLLOW;
        else if (context.ego_speed > context.obstacle_speed + 0.7)
            return DecisionType::OVERTAKE;
        else
            return DecisionType::IGNORE;
    }

    // 3. 让行超时？
    if (IsYieldTimeout(context))
        return DecisionType::FOLLOW;

    // 4. 保持让行
    return DecisionType::YIELD;
}

bool YieldState::ShouldStop(const DecisionContext & context) const
{
    // 停车条件：距离过近
    double distance = context.projection.distance;
    return distance < params_.stop_distance_threshold;
}

bool YieldState::IsClearToPass(const DecisionContext & context) const
{
    // 1. 有足够的安全时间
    // 2. 有足够的距离
    // 3. 自车速度可以安全通过
    double dx = context.projection.distance;
    double dy = context.projection.l;
    double total_distance = std::hypot(dx, dy);
    double ttc = context.projection.time_to_collision;

    bool time_condition = ttc > params_.clear_time_to_collision;
    bool distance_condition = total_distance > params_.yield_safety_distance * 2.0;
    bool speed_condition = context.ego_speed >= params_.min_passing_speed;
    return time_condition && distance_condition && speed_condition;
}

bool YieldState::IsYieldTimeout(const DecisionContext & context) const
{
    double elapsed = context.state_count * 0.1;  // 假设每帧0.1s
    return elapsed > params_.max_yield_time;
}

} // namespace Decision