// yield_state.h
#pragma once
#include <cmath>
#include <algorithm>

#include "local_planning/decision/decision_state_base.h"
#include "local_planning/decision/decision_state_factory.h"


namespace Decision
{
class YieldState : public DecisionState
{
public:
    struct YieldParams
    {
        double yield_safety_distance = 3.0;        // 让行安全距离(m)
        double max_yield_time = 5.0;               // 最大让行时间(s)
        double clear_time_to_collision = 8.0;      // 安全通过所需的最小碰撞时间(s)
        double stop_distance_threshold = 2.0;      // 停车距离阈值(m)
        double min_passing_speed = 1.0;            // 最小通过速度(m/s)
    };

public:
    YieldState();
    ~YieldState() override = default;

    // 禁止拷贝和移动
    YieldState(const YieldState &) = delete;
    YieldState & operator=(const YieldState &) = delete;
    YieldState(YieldState &&) = delete;
    YieldState & operator=(YieldState &&) = delete;

    // DecisionState接口实现
    DecisionType Evaluate(DecisionContext & context) override;
    std::string GetName() const override { return "YIELD"; }
    DecisionType GetType() const override { return DecisionType::YIELD; }

    // 参数访问
    void SetParams(const YieldParams & params) { params_ = params; }
    const YieldParams & GetParams() const { return params_; }

    // 状态检查
    bool IsClearToPass(const DecisionContext & context) const;
    bool ShouldStop(const DecisionContext & context) const;
    bool IsYieldTimeout(const DecisionContext & context) const;

private:
    YieldParams params_;
};

} // namespace Decision