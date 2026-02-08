// follow_state.h
#pragma once
#include <cmath>
#include <algorithm>

#include "local_planning/decision/decision_state_base.h"
#include "local_planning/decision/decision_state_factory.h"


namespace Decision
{
class FollowState : public DecisionState
{
public:
    struct FollowParams
    {
        double min_follow_distance = 5.0;          // 最小跟车距离(m)
        double max_follow_distance = 30.0;         // 最大跟车距离(m)
        double safe_time_headway = 2.0;            // 安全时距(s)
        double min_overtake_speed_gain = 2.0;      // 最小超车速度增益(m/s)
        double follow_speed_margin = 0.5;          // 跟车速度裕度(m/s)
        double emergency_stop_distance = 2.0;      // 紧急停车距离(m)
        double lateral_follow_threshold = 1.0;     // 横向跟车阈值(m)
    };

public:
    FollowState();
    ~FollowState() override = default;

    // 禁止拷贝和移动
    FollowState(const FollowState &) = delete;
    FollowState & operator=(const FollowState &) = delete;
    FollowState(FollowState &&) = delete;
    FollowState & operator=(FollowState &&) = delete;

    // DecisionState接口实现
    DecisionType Evaluate(DecisionContext & context) override;
    std::string GetName() const override { return "FOLLOW"; }
    DecisionType GetType() const override { return DecisionType::FOLLOW; }

    // 参数访问
    void SetParams(const FollowParams & params) { params_ = params; }
    const FollowParams & GetParams() const { return params_; }

    // 辅助函数
    double CalculateSafeFollowingDistance(double ego_speed, double obstacle_speed) const;

private:
    FollowParams params_;

    // 评估辅助函数
    bool ShouldStop(double distance) const;
    bool CanOvertake(const DecisionContext & context) const;
    bool ShouldYield(const DecisionContext & context) const;
    bool CanIgnore(const DecisionContext & context) const;
};

} // namespace Decision