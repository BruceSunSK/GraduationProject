// follow_state.cpp
#include "local_planning/decision/follow_state.h"


namespace Decision
{
FollowState::FollowState()
{
    // 设置默认参数
    params_.min_follow_distance = 5.0;
    params_.max_follow_distance = 30.0;
    params_.safe_time_headway = 2.0;
    params_.min_overtake_speed_gain = 2.0;
    params_.follow_speed_margin = 0.5;
    params_.emergency_stop_distance = 2.0;
    params_.lateral_follow_threshold = 1.0;

    // 初始化状态转移
    AddTransition(DecisionType::OVERTAKE, DecisionStateFactory::CreateState(DecisionType::OVERTAKE));
    AddTransition(DecisionType::YIELD, DecisionStateFactory::CreateState(DecisionType::YIELD));
    AddTransition(DecisionType::STOP, DecisionStateFactory::CreateState(DecisionType::STOP));
    AddTransition(DecisionType::IGNORE, DecisionStateFactory::CreateState(DecisionType::IGNORE));
}

DecisionType FollowState::Evaluate(DecisionContext & context)
{
    double distance = context.projection.s - context.ego_position.s;
    double lateral_offset = std::abs(context.projection.l);

    // 1. 检查是否需要停车
    if (ShouldStop(distance))
    {
        return DecisionType::STOP;
    }

    // 2. 检查是否可以超车
    if (CanOvertake(context))
    {
        return DecisionType::OVERTAKE;
    }

    // 3. 检查是否需要让行
    if (ShouldYield(context))
    {
        return DecisionType::YIELD;
    }

    // 4. 检查是否可以忽略
    if (CanIgnore(context))
    {
        return DecisionType::IGNORE;
    }

    // 5. 保持跟车状态
    return DecisionType::FOLLOW;
}

double FollowState::CalculateSafeFollowingDistance(double ego_speed, double obstacle_speed) const
{
    // 使用安全时距计算安全跟车距离
    // 距离 = 反应时间距离 + 安全余量
    const double reaction_time = 1.0;  // 反应时间(s)
    const double safety_margin = 2.0;  // 安全余量(m)

    double relative_speed = ego_speed - obstacle_speed;
    double stopping_distance = ego_speed * reaction_time +
        (ego_speed * ego_speed) / (2 * 3.0);  // 假设减速度为3m/s²

    return std::max(params_.min_follow_distance, stopping_distance + safety_margin);
}

bool FollowState::ShouldStop(double distance) const
{
    return distance < params_.emergency_stop_distance;
}

bool FollowState::CanOvertake(const DecisionContext & context) const
{
    double distance = context.projection.s - context.ego_position.s;
    double lateral_offset = std::abs(context.projection.l);
    double speed_diff = context.ego_speed - context.obstacle_speed;  // 使用context.obstacle_speed

    // 超车条件：
    // 1. 自车比前车快
    // 2. 距离较近
    // 3. 有足够的横向空间
    // 4. 不是紧急情况
    bool speed_condition = speed_diff > params_.min_overtake_speed_gain;
    bool distance_condition = distance < params_.min_follow_distance * 1.5;
    bool lateral_condition = lateral_offset > params_.lateral_follow_threshold &&
        lateral_offset < 3.0;
    bool emergency_condition = context.projection.time_to_collision > 1.0;

    return speed_condition && distance_condition && lateral_condition && emergency_condition;
}

bool FollowState::ShouldYield(const DecisionContext & context) const
{
    double time_to_collision = context.projection.time_to_collision;

    // 让行条件：碰撞时间过短
    return time_to_collision < params_.safe_time_headway / 2.0;
}

bool FollowState::CanIgnore(const DecisionContext & context) const
{
    double distance = context.projection.s - context.ego_position.s;
    double time_to_collision = context.projection.time_to_collision;

    // 忽略条件：距离过远或碰撞时间过长
    return distance > params_.max_follow_distance || time_to_collision > 10.0;
}

} // namespace Decision