// overtake_state.cpp
#include "local_planning/decision/overtake_state.h"


namespace Decision
{
OvertakeState::OvertakeState()
{
    // 设置默认参数
    params_.overtake_lateral_margin = 1.5;
    params_.overtake_safety_distance = 3.0;
    params_.overtake_completion_threshold = -5.0;
    params_.max_overtake_time = 10.0;
    params_.min_overtake_speed_gain = 2.0;
    params_.emergency_abort_ttc = 1.0;

    // 初始化状态转移
    AddTransition(DecisionType::FOLLOW, DecisionStateFactory::CreateState(DecisionType::FOLLOW));
    AddTransition(DecisionType::YIELD, DecisionStateFactory::CreateState(DecisionType::YIELD));
    AddTransition(DecisionType::STOP, DecisionStateFactory::CreateState(DecisionType::STOP));
    AddTransition(DecisionType::IGNORE, DecisionStateFactory::CreateState(DecisionType::IGNORE));
}

DecisionType OvertakeState::Evaluate(DecisionContext & context)
{
    // 1. 检查是否完成超车
    if (IsOvertakeCompleted(context))
    {
        return DecisionType::IGNORE;
    }

    // 2. 检查是否需要放弃超车
    if (ShouldAbortOvertake(context))
    {
        return DecisionType::FOLLOW;
    }

    // 3. 检查是否有紧急情况
    if (context.projection.time_to_collision < params_.emergency_abort_ttc)
    {
        return DecisionType::STOP;
    }

    // 4. 检查是否安全
    if (!IsSafeToOvertake(context))
    {
        return DecisionType::YIELD;
    }

    // 5. 检查是否超车时间过长
    if (context.state_count * 0.1 > params_.max_overtake_time)  // 假设每帧0.1秒
    {
        return DecisionType::FOLLOW;
    }

    // 6. 保持超车状态
    return DecisionType::OVERTAKE;
}

bool OvertakeState::IsOvertakeCompleted(const DecisionContext & context) const
{
    double relative_position = context.projection.s - context.ego_position.s;
    return relative_position < params_.overtake_completion_threshold;
}

bool OvertakeState::ShouldAbortOvertake(const DecisionContext & context) const
{
    double lateral_offset = std::abs(context.projection.l);
    double speed_gain = context.ego_speed - context.obstacle_speed;  // 使用context.obstacle_speed

    // 放弃超车条件：
    // 1. 横向距离不够
    // 2. 速度增益不足
    // 3. 对向来车或其他危险

    bool lateral_condition = lateral_offset < params_.overtake_lateral_margin / 2.0;
    bool speed_condition = speed_gain < params_.min_overtake_speed_gain / 2.0;

    return lateral_condition || speed_condition;
}

bool OvertakeState::IsSafeToOvertake(const DecisionContext & context) const
{
    // 安全检查：
    // 1. 有足够的横向空间
    // 2. 对向车道没有来车（这里简化处理）
    // 3. 前方视野清晰

    double lateral_offset = std::abs(context.projection.l);
    bool lateral_safe = lateral_offset > params_.overtake_lateral_margin;

    // 简化的视野检查：如果前方有其他障碍物，可能不安全
    bool forward_clear = true;  // 实际中需要检查前方是否有其他障碍物

    return lateral_safe && forward_clear;
}

} // namespace Decision