// ignore_state.h
#pragma once
#include <cmath>
#include <algorithm>

#include "local_planning/decision/decision_state_base.h"
#include "local_planning/decision/decision_state_factory.h"


namespace Decision
{
class IgnoreState : public DecisionState
{
public:
    // 忽略状态的评估参数
    struct IgnoreParams
    {
        double max_ignore_distance = 50.0;        // 最大忽略距离(m)
        double min_attention_distance = 10.0;     // 最小关注距离(m)
        double min_attention_ttc = 3.0;           // 最小需要关注的碰撞时间(s)
        double lateral_ignore_threshold = 3.0;    // 横向忽略阈值(m)
        double speed_difference_threshold = 5.0;  // 速度差阈值(m/s)
    };

public:
    IgnoreState();
    ~IgnoreState() override = default;

    // 禁止拷贝和移动
    IgnoreState(const IgnoreState &) = delete;
    IgnoreState & operator=(const IgnoreState &) = delete;
    IgnoreState(IgnoreState &&) = delete;
    IgnoreState & operator=(IgnoreState &&) = delete;

    // DecisionState接口实现
    DecisionType Evaluate(DecisionContext & context) override;
    std::string GetName() const override { return "IGNORE"; }
    DecisionType GetType() const override { return DecisionType::IGNORE; }

    // 参数访问
    void SetParams(const IgnoreParams & params) { params_ = params; }
    const IgnoreParams & GetParams() const { return params_; }

private:
    IgnoreParams params_;

    // 评估辅助函数
    bool ShouldIgnoreByDistance(double distance) const;
    bool ShouldIgnoreByLateralOffset(double lateral_offset) const;
    bool ShouldIgnoreBySpeedDifference(double speed_diff) const;
    bool ShouldIgnoreByTTC(double ttc) const;
};

} // namespace Decision