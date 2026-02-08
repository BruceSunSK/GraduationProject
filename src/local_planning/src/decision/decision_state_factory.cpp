// decision_state_factory.cpp
#include "local_planning/decision/decision_state_factory.h"

#include "local_planning/decision/ignore_state.h"
#include "local_planning/decision/follow_state.h"
#include "local_planning/decision/overtake_state.h"
#include "local_planning/decision/yield_state.h"
#include "local_planning/decision/stop_state.h"

namespace Decision
{

void DecisionStateFactory::Initialize()
{
    // 如果需要，可以在这里进行初始化
}

std::shared_ptr<DecisionState> DecisionStateFactory::CreateState(DecisionType type)
{
    switch (type)
    {
    case DecisionType::IGNORE:
        return std::make_shared<IgnoreState>();

    case DecisionType::FOLLOW:
        return std::make_shared<FollowState>();

    case DecisionType::OVERTAKE:
        return std::make_shared<OvertakeState>();

    case DecisionType::YIELD:
        return std::make_shared<YieldState>();

    case DecisionType::STOP:
        return std::make_shared<StopState>();

    case DecisionType::UNKNOWN:
        throw std::invalid_argument("Cannot create UNKNOWN state type");

    default:
        throw std::invalid_argument("Unknown decision state type: " +
            std::to_string(static_cast<int>(type)));
    }
}

std::string DecisionStateFactory::StateTypeToString(DecisionType type)
{
    switch (type)
    {
    case DecisionType::IGNORE:
        return "IGNORE";

    case DecisionType::FOLLOW:
        return "FOLLOW";

    case DecisionType::OVERTAKE:
        return "OVERTAKE";

    case DecisionType::YIELD:
        return "YIELD";

    case DecisionType::STOP:
        return "STOP";

    case DecisionType::UNKNOWN:
        return "UNKNOWN";

    default:
        return "INVALID";
    }
}

DecisionType DecisionStateFactory::StringToStateType(const std::string & name)
{
    std::string upper_name = name;
    std::transform(upper_name.begin(), upper_name.end(), upper_name.begin(), ::toupper);

    if (upper_name == "IGNORE")
        return DecisionType::IGNORE;

    if (upper_name == "FOLLOW")
        return DecisionType::FOLLOW;

    if (upper_name == "OVERTAKE")
        return DecisionType::OVERTAKE;

    if (upper_name == "YIELD")
        return DecisionType::YIELD;

    if (upper_name == "STOP")
        return DecisionType::STOP;

    if (upper_name == "UNKNOWN")
        return DecisionType::UNKNOWN;

    throw std::invalid_argument("Unknown decision state name: " + name);
}

std::string DecisionStateFactory::ScenarioToString(TrafficScenario scenario)
{
    switch (scenario)
    {
    case TrafficScenario::SAME_DIRECTION:
        return "SAME_DIRECTION";

    case TrafficScenario::OPPOSITE_DIRECTION:
        return "OPPOSITE_DIRECTION";

    case TrafficScenario::CROSS_TRAFFIC:
        return "CROSS_TRAFFIC";

    case TrafficScenario::UNKNOWN:
        return "UNKNOWN";

    default:
        return "INVALID";
    }
}

TrafficScenario DecisionStateFactory::StringToScenario(const std::string & name)
{
    std::string upper_name = name;
    std::transform(upper_name.begin(), upper_name.end(), upper_name.begin(), ::toupper);

    if (upper_name == "SAME_DIRECTION")
        return TrafficScenario::SAME_DIRECTION;

    if (upper_name == "OPPOSITE_DIRECTION")
        return TrafficScenario::OPPOSITE_DIRECTION;

    if (upper_name == "CROSS_TRAFFIC")
        return TrafficScenario::CROSS_TRAFFIC;

    if (upper_name == "UNKNOWN")
        return TrafficScenario::UNKNOWN;

    throw std::invalid_argument("Unknown scenario name: " + name);
}

std::vector<DecisionType> DecisionStateFactory::GetAllStateTypes()
{
    return {
        DecisionType::IGNORE,
        DecisionType::FOLLOW,
        DecisionType::OVERTAKE,
        DecisionType::YIELD,
        DecisionType::STOP
    };
}

bool DecisionStateFactory::IsValidStateType(DecisionType type)
{
    auto valid_types = GetAllStateTypes();
    return std::find(valid_types.begin(), valid_types.end(), type) != valid_types.end();
}

} // namespace Decision