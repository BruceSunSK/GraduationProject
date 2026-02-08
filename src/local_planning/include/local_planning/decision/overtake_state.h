// overtake_state.h
#pragma once
#include <cmath>
#include <algorithm>

#include "local_planning/decision/decision_state_base.h"
#include "local_planning/decision/decision_state_factory.h"


namespace Decision
{
class OvertakeState : public DecisionState
{
public:
    struct OvertakeParams
    {
        double overtake_lateral_margin = 1.5;      // 超车横向裕度(m)
        double overtake_safety_distance = 3.0;     // 超车安全距离(m)
        double overtake_completion_threshold = -5.0; // 完成超车的最小相对位置(m)
        double max_overtake_time = 10.0;           // 最大超车时间(s)
        double min_overtake_speed_gain = 2.0;      // 最小超车速度增益(m/s)
        double emergency_abort_ttc = 1.0;          // 紧急放弃的碰撞时间(s)
    };

public:
    OvertakeState();
    ~OvertakeState() override = default;

    // 禁止拷贝和移动
    OvertakeState(const OvertakeState &) = delete;
    OvertakeState & operator=(const OvertakeState &) = delete;
    OvertakeState(OvertakeState &&) = delete;
    OvertakeState & operator=(OvertakeState &&) = delete;

    // DecisionState接口实现
    DecisionType Evaluate(DecisionContext & context) override;
    std::string GetName() const override { return "OVERTAKE"; }
    DecisionType GetType() const override { return DecisionType::OVERTAKE; }

    // 参数访问
    void SetParams(const OvertakeParams & params) { params_ = params; }
    const OvertakeParams & GetParams() const { return params_; }

    // 状态检查
    bool IsOvertakeCompleted(const DecisionContext & context) const;
    bool ShouldAbortOvertake(const DecisionContext & context) const;
    bool IsSafeToOvertake(const DecisionContext & context) const;

private:
    OvertakeParams params_;
};

} // namespace Decision