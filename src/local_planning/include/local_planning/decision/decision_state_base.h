// decision_state_base.h
#pragma once
#include <map>
#include <memory>
#include <string>
#include <cmath>

#include "global_planning/path/utils.h"
#include "local_planning/decision/data_type.h"


namespace Decision
{

// 决策上下文
struct DecisionContext
{
    // 自车状态
    Path::PathNode ego_position;
    double ego_speed;
    double ego_acceleration;

    // 障碍物投影信息
    ObstacleProjection projection;

    // 障碍物属性
    int obstacle_type;           // 障碍物类型
    double obstacle_speed;       // 障碍物速度
    geometry_msgs::Vector3 obstacle_dimension; // 障碍物尺寸

    // 道路信息
    double lane_width;
    double road_width;

    // 决策历史
    DecisionType last_state;     // 使用合并后的DecisionType
    int state_count;             // 连续相同状态计数
    double state_time;           // 状态进入时间

    // 时间戳
    double timestamp;
};

// 决策状态基类
class DecisionState
{
public:
    virtual ~DecisionState() = default;

    virtual DecisionType Evaluate(DecisionContext & context) = 0;  // 返回类型改为DecisionType
    virtual std::string GetName() const = 0;
    virtual DecisionType GetType() const = 0;  // 返回类型改为DecisionType

    void AddTransition(DecisionType next_state, std::shared_ptr<DecisionState> state)
    {
        transitions_[next_state] = state;
    }

    std::shared_ptr<DecisionState> GetNextState(DecisionType next_state)
    {
        auto it = transitions_.find(next_state);
        if (it != transitions_.end())
        {
            return it->second;
        }
        return nullptr;
    }

    const std::map<DecisionType, std::shared_ptr<DecisionState>> & GetAllTransitions() const
    {
        return transitions_;
    }

protected:
    std::map<DecisionType, std::shared_ptr<DecisionState>> transitions_;  // 键类型改为DecisionType
};

} // namespace Decision