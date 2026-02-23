// data_type.h
#pragma once
#include <string>
#include <vector>
#include <sstream>
#include <iomanip>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Vector3.h>

#include "global_planning/path/utils.h"
#include "perception/PredictedTrajectory.h"


namespace Decision
{
// 决策类型枚举
enum class DecisionType
{
    UNKNOWN = 0,
    IGNORE = 1,
    FOLLOW = 2,
    OVERTAKE = 3,
    YIELD = 4,
    STOP = 5
};

// 障碍物标签
enum class ObstacleTag
{
    NONE = 0,
    BLOCKING = 1,
    OVERTAKING = 2,
    YIELDING = 3,
    IGNORED = 4
};

// 决策信息
struct DecisionInfo
{
    DecisionType type = DecisionType::UNKNOWN;
    ObstacleTag tag = ObstacleTag::NONE;
    double decision_confidence = 0.0;
    double target_speed = 0.0;
    double safety_distance = 2.0;
    double desired_lateral_offset = 0.0;
    bool is_emergency = false;

    std::string ToString() const
    {
        std::ostringstream oss;
        oss << "DecisionInfo{type=";
        switch (type)
        {
        case DecisionType::IGNORE:   oss << "IGNORE"; break;
        case DecisionType::FOLLOW:   oss << "FOLLOW"; break;
        case DecisionType::OVERTAKE: oss << "OVERTAKE"; break;
        case DecisionType::YIELD:    oss << "YIELD"; break;
        case DecisionType::STOP:     oss << "STOP"; break;
        default:                     oss << "UNKNOWN"; break;
        }
        oss << ", conf=" << std::fixed << std::setprecision(2) << decision_confidence
            << ", target_speed=" << target_speed
            << ", safety_distance=" << safety_distance
            << ", lateral_offset=" << desired_lateral_offset
            << ", emergency=" << (is_emergency ? "yes" : "no") << "}";
        return oss.str();
    }
};

// 障碍物投影结果
struct ObstacleProjection
{
    double s = 0.0;
    double l = 0.0;
    double width = 0.0;
    double length = 0.0;
    bool is_ahead = true;
    double time_to_collision = 999.0;
    double relative_speed = 0.0;
    double distance = 0.0;
    std::vector<std::pair<double, double>> predicted_s; // (t, s) 对

    std::string ToString() const
    {
        std::ostringstream oss;
        oss << "Projection{s=" << s << ", l=" << l
            << ", width=" << width << ", length=" << length
            << ", ahead=" << (is_ahead ? "true" : "false")
            << ", TTC=" << time_to_collision
            << ", rel_speed=" << relative_speed
            << ", dist=" << distance
            << ", pred_size=" << predicted_s.size() << "}";
        return oss.str();
    }
};

// 场景类型
enum class TrafficScenario
{
    SAME_DIRECTION = 0,
    OPPOSITE_DIRECTION = 1,
    CROSS_TRAFFIC = 2,
    UNKNOWN = 3
};

inline std::string ScenarioToString(TrafficScenario s)
{
    switch (s)
    {
    case TrafficScenario::SAME_DIRECTION:     return "SAME_DIRECTION";
    case TrafficScenario::OPPOSITE_DIRECTION: return "OPPOSITE_DIRECTION";
    case TrafficScenario::CROSS_TRAFFIC:      return "CROSS_TRAFFIC";
    default:                                  return "UNKNOWN";
    }
}

} // namespace Decision