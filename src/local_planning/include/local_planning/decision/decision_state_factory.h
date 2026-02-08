// decision_state_factory.h
#pragma once
#include <memory>
#include <string>
#include <stdexcept>
#include <algorithm>
#include <map>

#include "local_planning/decision/data_type.h"
#include "local_planning/decision/decision_state_base.h"


namespace Decision
{
// 决策状态工厂类
class DecisionStateFactory
{
public:
    DecisionStateFactory() = default;
    ~DecisionStateFactory() = default;

    // 禁止拷贝和移动
    DecisionStateFactory(const DecisionStateFactory &) = delete;
    DecisionStateFactory & operator=(const DecisionStateFactory &) = delete;
    DecisionStateFactory(DecisionStateFactory &&) = delete;
    DecisionStateFactory & operator=(DecisionStateFactory &&) = delete;

    // 创建状态实例
    static std::shared_ptr<DecisionState> CreateState(DecisionType type);

    // 状态类型转换
    static std::string StateTypeToString(DecisionType type);
    static DecisionType StringToStateType(const std::string & name);

    // 场景类型转换
    static std::string ScenarioToString(TrafficScenario scenario);
    static TrafficScenario StringToScenario(const std::string & name);

    // 获取所有可用状态类型
    static std::vector<DecisionType> GetAllStateTypes();

    // 验证状态类型是否有效
    static bool IsValidStateType(DecisionType type);

private:
    // 初始化（如果需要）
    static void Initialize();
};

} // namespace Decision