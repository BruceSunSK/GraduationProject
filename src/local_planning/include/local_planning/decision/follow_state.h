// follow_state.h
#pragma once
#include "local_planning/decision/decision_state_base.h"


namespace Decision
{
class FollowState : public DecisionState
{
public:
    struct FollowParams
    {
        double min_follow_distance = 5.0;
        double max_follow_distance = 30.0;
        double safe_time_headway = 2.5;
        double min_overtake_speed_gain = 1.0;
        double follow_speed_margin = 0.5;
        double emergency_stop_distance = 2.0;
        double lateral_follow_threshold = 1.0;
    };

public:
    FollowState() = default;
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

private:
    FollowParams params_;
    bool ShouldStop(const DecisionContext & context) const;
    bool CanOvertake(const DecisionContext & context) const;
    bool ShouldYield(const DecisionContext & context) const;
    bool CanIgnore(const DecisionContext & context) const;
};

} // namespace Decision