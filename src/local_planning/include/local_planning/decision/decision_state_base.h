// decision_state_base.h
#pragma once
#include <map>
#include <memory>
#include <string>
#include <cmath>

#include "global_planning/path/utils.h"
#include "global_planning/path/reference_path.h"
#include "global_planning/map/distance_map.h"
#include "local_planning/decision/data_type.h"


namespace Decision
{
// 决策上下文（每个障碍物独立）
struct DecisionContext
{
    // 自车状态
    Path::PathNode ego_position;
    double ego_speed;
    double ego_acceleration;

    // 障碍物信息
    int obstacle_id;
    int obstacle_type;
    double obstacle_speed;
    geometry_msgs::Vector3 obstacle_dimension;
    ObstacleProjection projection;

    // 道路环境（在当前障碍物投影点处）
    double curvature;          // 参考线曲率
    double cost_value;         // 代价地图值

    // 预测轨迹
    std::vector<perception::PredictedTrajectory> predicted_trajectory;

    // 车道信息
    double lane_width;
    double road_width;

    // 决策历史
    DecisionType last_state;
    int state_count;
    double state_time;         // 当前状态持续时间 (s)

    double timestamp;
};

// 决策状态基类
class DecisionState
{
public:
    virtual ~DecisionState() = default;

    // 评估当前上下文，返回建议的下一个状态
    virtual DecisionType Evaluate(DecisionContext & context) = 0;

    virtual std::string GetName() const = 0;
    virtual DecisionType GetType() const = 0;

    // 状态转移管理
    void AddTransition(DecisionType next_state, std::shared_ptr<DecisionState> state)
    {
        transitions_[next_state] = state;
    }

    std::shared_ptr<DecisionState> GetNextState(DecisionType next_state) const
    {
        auto it = transitions_.find(next_state);
        return (it != transitions_.end()) ? it->second : nullptr;
    }

    const std::map<DecisionType, std::shared_ptr<DecisionState>> & GetAllTransitions() const
    {
        return transitions_;
    }

protected:
    std::map<DecisionType, std::shared_ptr<DecisionState>> transitions_;
};

} // namespace Decision