// yield_state.cpp
#include "local_planning/decision/yield_state.h"


namespace Decision
{
DecisionType YieldState::Evaluate(DecisionContext & context)
{
    double distance = context.projection.s - context.ego_position.s;
    double ttc = context.projection.time_to_collision;

    // 1. 需要停车？
    if (ShouldStop(context))
        return DecisionType::STOP;

    // 2. 可以安全通过？
    if (IsClearToPass(context))
    {
        if (distance > 20.0)
            return DecisionType::FOLLOW;
        else if (context.ego_speed > context.obstacle_speed + 2.0)
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

bool YieldState::IsClearToPass(const DecisionContext & context) const
{
    double ttc = context.projection.time_to_collision;
    double distance = context.projection.s - context.ego_position.s;    // 可以通过的条件：
    // 1. 有足够的安全时间
    // 2. 有足够的距离
    // 3. 自车速度可以安全通过

    bool time_condition = ttc > params_.clear_time_to_collision;
    bool distance_condition = distance > params_.yield_safety_distance * 2.0;
    bool speed_condition = context.ego_speed >= params_.min_passing_speed;
    return time_condition && distance_condition && speed_condition;
}

bool YieldState::ShouldStop(const DecisionContext & context) const
{
    double distance = context.projection.s - context.ego_position.s;
    // 停车条件：距离过近
    return distance < params_.stop_distance_threshold;
}

bool YieldState::IsYieldTimeout(const DecisionContext & context) const
{
    double elapsed = context.state_count * 0.1;  // 假设每帧0.1s
    return elapsed > params_.max_yield_time;
}

} // namespace Decision