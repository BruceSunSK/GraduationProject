// stop_state.cpp
#include "local_planning/decision/stop_state.h"


namespace Decision
{
DecisionType StopState::Evaluate(DecisionContext & context)
{
    double distance = context.projection.s - context.ego_position.s;
    double ttc = context.projection.time_to_collision;
    double lateral_offset = std::abs(context.projection.l);

    // 1. 障碍物已远离？
    if (IsObstacleCleared(context))
        return DecisionType::FOLLOW;

    // 2. 可以安全通过？
    if (CanPassSafely(context))
    {
        if (lateral_offset > params_.lateral_passing_threshold)
            return DecisionType::OVERTAKE;
        else
            return DecisionType::FOLLOW;
    }

    // 3. 改变策略？
    if (ShouldChangeStrategy(context))
        return DecisionType::YIELD;

    // 4. 超时？
    double elapsed = context.state_count * 0.1;
    if (elapsed > params_.max_stop_time)
        return DecisionType::YIELD;

    // 5. 保持停车
    return DecisionType::STOP;
}

bool StopState::IsObstacleCleared(const DecisionContext & context) const
{
    double distance = context.projection.s - context.ego_position.s;
    return distance > params_.safe_clear_distance;
}

bool StopState::CanPassSafely(const DecisionContext & context) const
{
    double ttc = context.projection.time_to_collision;
    double distance = context.projection.s - context.ego_position.s;

    // 可以通过的条件：
    // 1. 有足够的安全时间
    // 2. 有足够的距离
    return ttc > params_.min_passing_ttc && distance > 5.0;
}

bool StopState::ShouldChangeStrategy(const DecisionContext & context) const
{
    double elapsed = context.state_count * 0.1;
    double ttc = context.projection.time_to_collision;
    // 改变策略的条件：
    // 1. 停车时间较长但障碍物仍在
    // 2. 碰撞时间仍然很短
    return elapsed > params_.max_stop_time / 2.0 && ttc < 2.0;
}

} // namespace Decision