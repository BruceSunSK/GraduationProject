// yield_state.cpp
#include "local_planning/decision/yield_state.h"


namespace Decision
{
YieldState::YieldState()
{
    // 设置默认参数
    params_.yield_safety_distance = 3.0;
    params_.max_yield_time = 5.0;
    params_.clear_time_to_collision = 8.0;
    params_.stop_distance_threshold = 2.0;
    params_.min_passing_speed = 1.0;

    // 初始化状态转移
    AddTransition(DecisionType::FOLLOW, DecisionStateFactory::CreateState(DecisionType::FOLLOW));
    AddTransition(DecisionType::OVERTAKE, DecisionStateFactory::CreateState(DecisionType::OVERTAKE));
    AddTransition(DecisionType::STOP, DecisionStateFactory::CreateState(DecisionType::STOP));
    AddTransition(DecisionType::IGNORE, DecisionStateFactory::CreateState(DecisionType::IGNORE));
}

DecisionType YieldState::Evaluate(DecisionContext & context)
{
    double distance = context.projection.s - context.ego_position.s;
    double time_to_collision = context.projection.time_to_collision;

    // 1. 检查是否需要停车
    if (ShouldStop(context))
    {
        return DecisionType::STOP;
    }

    // 2. 检查是否可以安全通过
    if (IsClearToPass(context))
    {
        // 根据情况选择下一步状态
        if (distance > 20.0)  // 障碍物较远
        {
            return DecisionType::FOLLOW;
        }
        else if (context.ego_speed > context.obstacle_speed + 2.0)  // 有机会超车
        {
            return DecisionType::OVERTAKE;
        }
        else
        {
            return DecisionType::IGNORE;
        }
    }

    // 3. 检查是否让行时间过长
    if (IsYieldTimeout(context))
    {
        // 考虑改变策略
        return DecisionType::FOLLOW;
    }

    // 4. 保持让行状态
    return DecisionType::YIELD;
}

bool YieldState::IsClearToPass(const DecisionContext & context) const
{
    double time_to_collision = context.projection.time_to_collision;
    double distance = context.projection.s - context.ego_position.s;

    // 可以通过的条件：
    // 1. 有足够的安全时间
    // 2. 有足够的距离
    // 3. 自车速度可以安全通过

    bool time_condition = time_to_collision > params_.clear_time_to_collision;
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
    // 假设每帧0.1秒，计算实际时间
    double elapsed_time = context.state_count * 0.1;
    return elapsed_time > params_.max_yield_time;
}

} // namespace Decision