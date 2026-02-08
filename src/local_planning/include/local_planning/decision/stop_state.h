// stop_state.h
#pragma once
#include <cmath>
#include <algorithm>

#include "local_planning/decision/decision_state_base.h"
#include "local_planning/decision/decision_state_factory.h"


namespace Decision
{
class StopState : public DecisionState
{
public:
    struct StopParams
    {
        double safe_clear_distance = 10.0;     // 安全通过距离(m)
        double max_stop_time = 3.0;            // 最大停车时间(s)
        double min_passing_ttc = 5.0;          // 最小通过碰撞时间(s)
        double lateral_passing_threshold = 1.0; // 横向通过阈值(m)
    };

public:
    StopState();
    ~StopState() override = default;

    // 禁止拷贝和移动
    StopState(const StopState &) = delete;
    StopState & operator=(const StopState &) = delete;
    StopState(StopState &&) = delete;
    StopState & operator=(StopState &&) = delete;

    // DecisionState接口实现
    DecisionType Evaluate(DecisionContext & context) override;
    std::string GetName() const override { return "STOP"; }
    DecisionType GetType() const override { return DecisionType::STOP; }

    // 参数访问
    void SetParams(const StopParams & params) { params_ = params; }
    const StopParams & GetParams() const { return params_; }

    // 状态检查
    bool IsObstacleCleared(const DecisionContext & context) const;
    bool CanPassSafely(const DecisionContext & context) const;
    bool ShouldChangeStrategy(const DecisionContext & context) const;

private:
    StopParams params_;
};

} // namespace Decision