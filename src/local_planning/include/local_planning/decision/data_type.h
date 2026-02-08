// data_type.h
#pragma once
#include <string>
#include <vector>
#include <memory>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Vector3.h>
#include "global_planning/path/utils.h"

namespace Decision
{
// 合并后的决策类型枚举，同时用于状态机和障碍物标签
enum class DecisionType
{
    UNKNOWN = 0,
    IGNORE = 1,      // 忽略
    FOLLOW = 2,      // 跟车
    OVERTAKE = 3,    // 超车
    YIELD = 4,       // 让行
    STOP = 5         // 停车
};

// 决策信息结构体
struct DecisionInfo
{
    DecisionType type = DecisionType::UNKNOWN;
    double decision_confidence = 0.0;      // 决策置信度
    double target_speed = 0.0;             // 目标速度
    double safety_distance = 2.0;          // 安全距离
    double desired_lateral_offset = 0.0;   // 期望横向偏移
    bool is_emergency = false;             // 是否紧急情况

    std::string ToString() const
    {
        std::string type_str;
        switch (type)
        {
        case DecisionType::IGNORE: type_str = "IGNORE"; break;
        case DecisionType::FOLLOW: type_str = "FOLLOW"; break;
        case DecisionType::OVERTAKE: type_str = "OVERTAKE"; break;
        case DecisionType::YIELD: type_str = "YIELD"; break;
        case DecisionType::STOP: type_str = "STOP"; break;
        default: type_str = "UNKNOWN"; break;
        }
        return "DecisionInfo{type=" + type_str +
            ", confidence=" + std::to_string(decision_confidence) +
            ", target_speed=" + std::to_string(target_speed) +
            ", safety_distance=" + std::to_string(safety_distance) + "}";
    }
};

// 障碍物投影结果
struct ObstacleProjection
{
    double s = 0.0;                   // 沿参考线的纵向距离
    double l = 0.0;                   // 横向偏移
    double width = 0.0;               // 在参考线上的投影宽度
    double length = 0.0;              // 在参考线上的投影长度
    bool is_ahead = true;             // 是否在自车前方
    double time_to_collision = 999.0; // 预计碰撞时间(s)
    double relative_speed = 0.0;      // 相对速度(正表示自车更快)
    double distance = 0.0;            // 欧氏距离

    std::string ToString() const
    {
        return "Projection{s=" + std::to_string(s) +
            ", l=" + std::to_string(l) +
            ", TTC=" + std::to_string(time_to_collision) +
            ", ahead=" + (is_ahead ? "true" : "false") + "}";
    }
};

// 场景类型枚举
enum class TrafficScenario
{
    SAME_DIRECTION = 0,      // 同向交通
    OPPOSITE_DIRECTION = 1,  // 对向交通
    CROSS_TRAFFIC = 2,       // 交叉交通
    UNKNOWN = 3
};

} // namespace Decision