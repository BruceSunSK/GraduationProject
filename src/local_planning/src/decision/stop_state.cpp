// stop_state.cpp
#include "local_planning/decision/stop_state.h"


namespace Decision
{
StopState::StopState()
{
    // 设置默认参数
    params_.safe_clear_distance = 10.0;
    params_.max_stop_time = 3.0;
    params_.min_passing_ttc = 5.0;
    params_.lateral_passing_threshold = 1.0;

    // 初始化状态转移
    AddTransition(DecisionType::FOLLOW, DecisionStateFactory::CreateState(DecisionType::FOLLOW));
    AddTransition(DecisionType::OVERTAKE, DecisionStateFactory::CreateState(DecisionType::OVERTAKE));
    AddTransition(DecisionType::YIELD, DecisionStateFactory::CreateState(DecisionType::YIELD));
    AddTransition(DecisionType::IGNORE, DecisionStateFactory::CreateState(DecisionType::IGNORE));
}

DecisionType StopState::Evaluate(DecisionContext & context)
{
    double distance = context.projection.s - context.ego_position.s;
    double time_to_collision = context.projection.time_to_collision;
    double lateral_offset = std::abs(context.projection.l);

    // 1. 检查障碍物是否已经远离
    if (IsObstacleCleared(context))
    {
        return DecisionType::FOLLOW;
    }

    // 2. 检查是否可以安全通过
    if (CanPassSafely(context))
    {
        if (lateral_offset > params_.lateral_passing_threshold)  // 有超车空间
        {
            return DecisionType::OVERTAKE;
        }
        else  // 跟车
        {
            return DecisionType::FOLLOW;
        }
    }

    // 3. 检查是否应该改变策略
    if (ShouldChangeStrategy(context))
    {
        return DecisionType::YIELD;
    }

    // 4. 检查是否停车时间过长
    double elapsed_time = context.state_count * 0.1;  // 假设每帧0.1秒
    if (elapsed_time > params_.max_stop_time)
    {
        // 考虑绕行或其他策略
        return DecisionType::YIELD;
    }

    // 5. 保持停车状态
    return DecisionType::STOP;
}

bool StopState::IsObstacleCleared(const DecisionContext & context) const
{
    double distance = context.projection.s - context.ego_position.s;
    return distance > params_.safe_clear_distance;
}

bool StopState::CanPassSafely(const DecisionContext & context) const
{
    double time_to_collision = context.projection.time_to_collision;
    double distance = context.projection.s - context.ego_position.s;

    // 可以通过的条件：
    // 1. 有足够的安全时间
    // 2. 有足够的距离
    return time_to_collision > params_.min_passing_ttc && distance > 5.0;
}

bool StopState::ShouldChangeStrategy(const DecisionContext & context) const
{
    double elapsed_time = context.state_count * 0.1;  // 假设每帧0.1秒
    double time_to_collision = context.projection.time_to_collision;

    // 改变策略的条件：
    // 1. 停车时间较长但障碍物仍在
    // 2. 碰撞时间仍然很短
    return elapsed_time > params_.max_stop_time / 2.0 && time_to_collision < 2.0;
}

} // namespace Decision