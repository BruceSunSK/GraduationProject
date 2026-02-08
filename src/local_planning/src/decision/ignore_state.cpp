// ignore_state.cpp
#include "local_planning/decision/ignore_state.h"


namespace Decision
{
IgnoreState::IgnoreState()
{
    // 设置默认参数
    params_.max_ignore_distance = 50.0;
    params_.min_attention_distance = 10.0;
    params_.min_attention_ttc = 3.0;
    params_.lateral_ignore_threshold = 3.0;
    params_.speed_difference_threshold = 5.0;

    // 初始化状态转移
    AddTransition(DecisionType::FOLLOW, DecisionStateFactory::CreateState(DecisionType::FOLLOW));
    AddTransition(DecisionType::OVERTAKE, DecisionStateFactory::CreateState(DecisionType::OVERTAKE));
    AddTransition(DecisionType::YIELD, DecisionStateFactory::CreateState(DecisionType::YIELD));
    AddTransition(DecisionType::STOP, DecisionStateFactory::CreateState(DecisionType::STOP));
}

DecisionType IgnoreState::Evaluate(DecisionContext & context)
{
    // 如果障碍物不在前方，保持忽略
    if (!context.projection.is_ahead)
    {
        return DecisionType::IGNORE;
    }

    double distance = context.projection.s - context.ego_position.s;
    double lateral_offset = std::abs(context.projection.l);
    double speed_diff = std::abs(context.ego_speed - context.obstacle_speed);  // 使用context.obstacle_speed
    double time_to_collision = context.projection.time_to_collision;

    // 检查是否应该继续忽略
    bool should_ignore = true;

    if (!ShouldIgnoreByDistance(distance))
        should_ignore = false;
    else if (!ShouldIgnoreByLateralOffset(lateral_offset))
        should_ignore = false;
    else if (!ShouldIgnoreBySpeedDifference(speed_diff))
        should_ignore = false;
    else if (!ShouldIgnoreByTTC(time_to_collision))
        should_ignore = false;

    if (should_ignore)
    {
        return DecisionType::IGNORE;
    }

    // 需要从忽略状态转移，根据场景选择下一个状态
    if (time_to_collision < 2.0)  // 紧急情况
    {
        return DecisionType::STOP;
    }
    else if (time_to_collision < 5.0)  // 需要关注
    {
        if (lateral_offset < 1.0)  // 同车道
        {
            return DecisionType::FOLLOW;
        }
        else if (lateral_offset < 2.5)  // 相邻车道
        {
            return DecisionType::OVERTAKE;
        }
        else  // 对向或交叉
        {
            return DecisionType::YIELD;
        }
    }
    else if (distance < params_.min_attention_distance)
    {
        // 距离很近但还有时间
        return DecisionType::FOLLOW;
    }

    // 默认保持忽略
    return DecisionType::IGNORE;
}

bool IgnoreState::ShouldIgnoreByDistance(double distance) const
{
    return distance > params_.max_ignore_distance;
}

bool IgnoreState::ShouldIgnoreByLateralOffset(double lateral_offset) const
{
    return lateral_offset > params_.lateral_ignore_threshold;
}

bool IgnoreState::ShouldIgnoreBySpeedDifference(double speed_diff) const
{
    return speed_diff > params_.speed_difference_threshold;
}

bool IgnoreState::ShouldIgnoreByTTC(double ttc) const
{
    return ttc > params_.min_attention_ttc;
}

} // namespace Decision