// follow_state.cpp
#include "local_planning/decision/follow_state.h"


namespace Decision
{
DecisionType FollowState::Evaluate(DecisionContext & context)
{
    if (ShouldStop(context))
        return DecisionType::STOP;

    if (CanOvertake(context))
        return DecisionType::OVERTAKE;

    if (ShouldYield(context))
        return DecisionType::YIELD;

    if (CanIgnore(context))
        return DecisionType::IGNORE;

    return DecisionType::FOLLOW;
}

bool FollowState::ShouldStop(const DecisionContext & context) const
{
    double distance = context.projection.distance;      // 障碍物投影纵向碰撞距离，已经考虑了自车、他车的长度
    return distance < params_.emergency_stop_distance;
}

bool FollowState::CanOvertake(const DecisionContext & context) const
{
    double distance = context.projection.distance;
    double lateral_offset = std::abs(context.projection.l);
    double speed_diff = context.ego_speed - context.obstacle_speed;

    bool speed_ok = speed_diff > params_.min_overtake_speed_gain;
    bool dist_ok = distance < params_.min_follow_distance * 1.5;
    // bool lateral_ok = lateral_offset > params_.lateral_follow_threshold && lateral_offset < 3.0;
    bool lateral_ok = lateral_offset < params_.lateral_follow_threshold;
    bool safe = context.projection.time_to_collision > 1.0;

    return speed_ok && dist_ok && lateral_ok && safe;
}

bool FollowState::ShouldYield(const DecisionContext & context) const
{
    return context.projection.time_to_collision < params_.safe_time_headway * 2.0;
}

bool FollowState::CanIgnore(const DecisionContext & context) const
{
    double distance = context.projection.distance;
    return distance > params_.max_follow_distance || context.projection.time_to_collision > 15.0;
}

} // namespace Decision