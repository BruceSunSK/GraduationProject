// decision_maker.h
#pragma once
#include <vector>
#include <map>
#include <string>
#include <sstream>
#include <memory>
#include <mutex>
#include <chrono>
#include <cmath>
#include <algorithm>
#include <limits>
#include <iomanip>

#include "global_planning/path/reference_path.h"
#include "local_planning/decision/data_type.h"
#include "local_planning/decision/decision_state_base.h"
#include "local_planning/obstacles/obstacle.h"


namespace Decision
{
// 障碍物决策历史
struct ObstacleDecisionHistory
{
    DecisionType current_state = DecisionType::IGNORE;
    DecisionType previous_state = DecisionType::IGNORE;
    int state_count = 0;                     // 连续相同状态计数
    std::chrono::steady_clock::time_point state_start_time; // 状态开始时间
    std::vector<DecisionType> state_history; // 状态历史（用于滤波）

    // 决策统计
    int ignore_count = 0;
    int follow_count = 0;
    int overtake_count = 0;
    int yield_count = 0;
    int stop_count = 0;

    void UpdateState(DecisionType new_state);
    double GetStateDuration() const;
    double GetStateConfidence() const;
    std::string GetHistoryString() const;
};

// 决策结果包装器
struct DecisionResult
{
    DecisionType decision_state = DecisionType::IGNORE;
    DecisionInfo decision_info;    // 转换为障碍物决策信息
    double confidence = 0.0;                 // 决策置信度
    std::string reasoning;                   // 决策理由

    bool IsValid() const { return decision_state != DecisionType::UNKNOWN; }
    std::string ToString() const;
};

// 决策器
class DecisionMaker
{
public:
    // 决策参数
    struct DecisionMakerParams
    {
        // 距离阈值
        double safe_distance = 2.0;              // 安全距离(m)
        double min_follow_distance = 5.0;        // 最小跟车距离(m)
        double max_follow_distance = 30.0;       // 最大跟车距离(m)
        double overtake_lateral_margin = 1.5;    // 超车横向裕度(m)

        // 时间阈值
        double safe_time_headway = 2.0;          // 安全时距(s)
        double min_time_to_collision = 3.0;      // 最小碰撞时间(s)
        double max_time_to_collision = 10.0;     // 最大碰撞时间(s)

        // 速度阈值
        double speed_tolerance = 1.0;            // 速度容忍度(m/s)
        double min_overtake_speed_gain = 2.0;    // 最小超车速度增益(m/s)

        // 决策稳定性
        int decision_hysteresis_count = 3;       // 决策迟滞计数
        double decision_filter_time = 0.5;       // 决策滤波时间(s)

        // 道路参数
        double default_lane_width = 3.5;         // 默认车道宽度(m)
        double road_boundary_margin = 0.5;       // 道路边界裕度(m)

        // 场景识别参数
        double cross_traffic_threshold = 1.5;    // 交叉交通横向偏移阈值(m)
        double same_direction_threshold = 1.0;   // 同向交通横向偏移阈值(m)
        double opposite_direction_threshold = -1.0; // 对向交通阈值(m)

        // 更新参数
        double update_frequency = 10.0;          // 决策更新频率(Hz)
        bool enable_debug_output = true;         // 启用调试输出
    };

public:
    DecisionMaker();
    ~DecisionMaker() = default;

    // 禁止拷贝和移动
    DecisionMaker(const DecisionMaker &) = delete;
    DecisionMaker & operator=(const DecisionMaker &) = delete;
    DecisionMaker(DecisionMaker &&) = delete;
    DecisionMaker & operator=(DecisionMaker &&) = delete;

    // 初始化
    void Initialize(const DecisionMakerParams & params);
    bool IsInitialized() const { return is_initialized_; }

    // 更新数据并决策
    void UpdateAndDecide(const Obstacle::Obstacle::List & obstacles,
        const Path::ReferencePath::Ptr & reference_path,
        const Path::PathNode & ego_position,
        double ego_speed,
        double ego_acceleration = 0.0);

    // 获取决策结果
    const Obstacle::Obstacle::List & GetObstaclesWithDecision() const { return obstacles_with_decision_; }
    const std::map<int, DecisionResult> & GetDecisionResults() const { return decision_results_; }
    const DecisionMakerParams & GetParams() const { return params_; }

    // 获取特定障碍物的决策
    bool GetObstacleDecision(int obstacle_id, DecisionResult & result) const;
    DecisionType GetObstacleDecisionState(int obstacle_id) const;

    // 场景识别
    TrafficScenario IdentifyScenario(const Obstacle::Obstacle::Ptr & obstacle) const;
    std::string GetScenarioString(TrafficScenario scenario) const;

    // 调试信息
    std::string GetDebugInfo() const;
    std::string GetStatistics() const;
    void ClearDebugInfo();

    // 边界生成接口（供规划器调用）
    struct PathBoundary
    {
        std::vector<std::array<std::pair<double, double>, 3>> bounds; // 三碰撞圆的边界
        std::vector<double> s_coordinates; // 对应的s坐标
    };

    PathBoundary GeneratePathBoundary(const std::vector<Path::PathNode> & ref_points) const;

    struct SpeedBoundary
    {
        std::vector<std::pair<double, double>> st_lower_bound; // ST下界
        std::vector<std::pair<double, double>> st_upper_bound; // ST上界
        std::vector<double> time_points;      // 时间点(s)
        std::vector<double> s_points;         // 距离点(m)
    };

    SpeedBoundary GenerateSpeedBoundary(double planning_horizon = 5.0,
        double time_resolution = 0.1) const;

private:
    // 处理单个障碍物
    DecisionResult ProcessObstacle(const Obstacle::Obstacle::Ptr & obstacle);

    // 计算障碍物投影和相关信息
    void CalculateObstacleInfo(Obstacle::Obstacle::Ptr obstacle);

    // 创建决策上下文
    DecisionContext CreateDecisionContext(const Obstacle::Obstacle::Ptr & obstacle) const;

    // 决策状态机执行
    DecisionType ExecuteStateMachine(const Obstacle::Obstacle::Ptr & obstacle,
        DecisionContext & context);

    // 应用决策结果到障碍物
    void ApplyDecisionToObstacle(Obstacle::Obstacle::Ptr obstacle,
        const DecisionResult & decision_result);

    // 决策滤波和稳定性处理
    DecisionType ApplyDecisionFilter(int obstacle_id, DecisionType new_state);
    DecisionType ApplyHysteresis(int obstacle_id, DecisionType new_state);

    // 边界生成辅助函数
    std::array<std::pair<double, double>, 3> CalculateObstacleBoundary(
        const Obstacle::Obstacle::Ptr & obstacle,
        const Path::PathNode & ref_point) const;

    std::pair<double, double> CalculateSTBoundaryForObstacle(
        const Obstacle::Obstacle::Ptr & obstacle,
        double time, double ego_speed) const;

    // 调试输出
    void AddDebugInfo(const std::string & info);
    void AddObstacleDebugInfo(const Obstacle::Obstacle::Ptr & obstacle,
        const DecisionResult & result);

private:
    DecisionMakerParams params_;
    bool is_initialized_ = false;

    // 当前帧数据
    Obstacle::Obstacle::List obstacles_with_decision_;
    Path::ReferencePath::Ptr reference_path_;
    Path::PathNode ego_position_;
    double ego_speed_ = 0.0;
    double ego_acceleration_ = 0.0;
    std::chrono::steady_clock::time_point last_update_time_;

    // 决策历史
    std::map<int, ObstacleDecisionHistory> obstacle_histories_;
    std::map<int, DecisionResult> decision_results_;

    // 状态机实例
    std::map<DecisionType, std::shared_ptr<DecisionState>> state_machines_;

    // 调试信息
    mutable std::stringstream debug_info_;
    mutable std::mutex debug_mutex_;

    // 统计信息
    struct Statistics
    {
        int total_frames = 0;
        int total_obstacles_processed = 0;
        std::map<DecisionType, int> state_counts;
        std::map<TrafficScenario, int> scenario_counts;
    };

    Statistics statistics_;
};

} // namespace Decision