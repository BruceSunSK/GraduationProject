// decision_maker.h
#pragma once
#include <vector>
#include <deque>
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
#include "global_planning/map/distance_map.h"
#include "local_planning/decision/data_type.h"
#include "local_planning/decision/decision_state_base.h"
#include "local_planning/obstacles/obstacle.h"

namespace Decision
{
// 单个障碍物的决策历史
struct ObstacleDecisionHistory
{
    DecisionType current_state = DecisionType::IGNORE;          // 当前最终决策状态（经过迟滞）
    DecisionType previous_state = DecisionType::IGNORE;         // 上一帧最终决策状态
    int state_count = 0;                                        // 当前最终状态连续帧数
    std::chrono::steady_clock::time_point state_start_time;     // 当前状态开始时间
    std::deque<DecisionType> state_history;                     // 最终决策历史（用于置信度计算）
    std::deque<DecisionType> raw_state_history;                 // 状态机原始输出历史（用于迟滞判断）
    TrafficScenario last_scenario = TrafficScenario::UNKNOWN;   // 上一帧识别的场景

    void UpdateState(DecisionType new_state);
    double GetStateDuration() const;
    double GetStateConfidence() const;
    std::string GetHistoryString() const;
};

// 决策结果（最终附着于障碍物）
struct DecisionResult
{
    DecisionType decision_state = DecisionType::IGNORE;
    DecisionInfo decision_info;
    double confidence = 0.0;
    std::string reasoning;

    bool IsValid() const { return decision_state != DecisionType::UNKNOWN; }
    std::string ToString() const;
};

// 路径边界（供路径QP使用）
struct PathBoundary
{
    std::vector<double> s_coordinates;
    std::vector<std::array<std::pair<double, double>, 3>> bounds; // 三碰撞圆边界
};

// 速度边界（供速度QP使用）
struct SpeedBoundary
{
    std::vector<double> time_points;                       // 时间点序列 (s)
    std::vector<std::pair<double, double>> bounds;         // 每个时间点的 (s_lower, s_upper)
};

// 决策器参数
struct DecisionMakerParams
{
    // 距离阈值
    double ego_half_length = 1.9;
    double safe_distance = 1.0;
    double min_follow_distance = 5.0;
    double max_follow_distance = 30.0;
    double overtake_lateral_margin = 4.0;
    double overtake_completion_threshold = 4.0;
    // 时间阈值
    double safe_time_headway = 2.0;
    double min_time_to_collision = 3.0;
    double max_time_to_collision = 10.0;
    // 速度阈值
    double speed_tolerance = 1.0;
    double min_overtake_speed_gain = 2.0;
    // 决策稳定性
    int decision_hysteresis_count = 3;          // 迟滞计数（连续几帧原始建议一致才切换）
    double decision_filter_time = 0.5;
    // 道路参数
    double default_lane_width = 3.5;
    double road_boundary_margin = 0.5;
    // 场景识别（基于角度的阈值，在 IdentifyScenario 中使用）
    double angle_same_threshold = 30.0 * M_PI / 180.0;      // 同向角度阈值
    double angle_opposite_threshold = 150.0 * M_PI / 180.0; // 对向角度阈值
    double angle_cross_low = 60.0 * M_PI / 180.0;           // 交叉角度下限
    double angle_cross_high = 120.0 * M_PI / 180.0;         // 交叉角度上限
    // 车辆参数
    double vehicle_min_speed = 0.0;   // 车辆最小速度
    double vehicle_max_speed = 8.0;   // 车辆最大速度
    // 调试
    bool enable_debug_output = true;
};

// 主决策器
class DecisionMaker
{
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

    // 设置代价地图、参考线（需在决策前设置）
    void SetCostMap(const Map::MultiMap::Ptr & cost_map) { cost_map_ = cost_map; }
    void SetReferencePath(const Path::ReferencePath::Ptr & reference_path) { reference_path_ = reference_path; }
    
    // 更新并决策（核心入口）
    void UpdateAndDecide(const Obstacle::Obstacle::List & obstacles,
                         const Path::PathNode & ego_position,
                         double ego_speed,
                         double ego_acceleration = 0.0);

    // 获取决策结果
    const Obstacle::Obstacle::List & GetObstaclesWithDecision() const { return obstacles_with_decision_; }
    const std::map<int, DecisionResult> & GetDecisionResults() const { return decision_results_; }
    DecisionType GetGlobalIntention() const { return global_intention_; }

    // 边界生成接口（供规划器调用）
    PathBoundary GeneratePathBoundary(const std::vector<Path::PathNode> & ref_points) const;
    SpeedBoundary GenerateSpeedBoundary(double planning_horizon = 5.0,
                                        double time_resolution = 0.1) const;
    std::vector<std::pair<double, double>> GenerateVConstraint(
        const std::vector<double> & time_points);

    // 调试
    std::string GetDebugInfo() const;
    std::string GetStatistics() const;
    void ClearDebugInfo();

private:
    // 处理单个障碍物
    DecisionResult ProcessObstacle(const Obstacle::Obstacle::Ptr & obstacle);

    // 计算障碍物投影及相关信息
    void CalculateObstacleInfo(Obstacle::Obstacle::Ptr obstacle);

    // 创建决策上下文
    DecisionContext CreateDecisionContext(const Obstacle::Obstacle::Ptr & obstacle) const;

    // 状态机执行（根据场景和历史选择合适的状态机入口）
    DecisionType ExecuteStateMachine(const Obstacle::Obstacle::Ptr & obstacle,
                                     DecisionContext & context,
                                     const TrafficScenario & scenario,
                                     bool is_new_obstacle,
                                     bool scenario_changed);

    // 应用决策到障碍物
    void ApplyDecisionToObstacle(Obstacle::Obstacle::Ptr obstacle,
                                 const DecisionResult & decision_result);

    // 决策迟滞（基于原始建议的连续次数）
    DecisionType ApplyHysteresis(int obstacle_id, DecisionType new_state);

    // 场景识别
    TrafficScenario IdentifyScenario(const Obstacle::Obstacle::Ptr & obstacle) const;

    // 边界生成辅助
    std::array<std::pair<double, double>, 3> CalculateObstacleBoundary(
        const Obstacle::Obstacle::Ptr & obstacle,
        const Path::PathNode & ref_point) const;
    std::pair<double, double> CalculateSTBoundaryForObstacle(
        const Obstacle::Obstacle::Ptr & obstacle,
        double time) const;   // 使用预测轨迹

    // 调试输出
    void AddDebugInfo(const std::string & info);
    void AddObstacleDebugInfo(const Obstacle::Obstacle::Ptr & obstacle,
                              const DecisionResult & result);

private:
    DecisionMakerParams params_;
    bool is_initialized_ = false;

    // 当前帧数据
    Map::MultiMap::Ptr cost_map_;
    Path::ReferencePath::Ptr reference_path_;
    Path::PathNode ego_position_;
    double ego_speed_ = 0.0;
    double ego_acceleration_ = 0.0;
    std::chrono::steady_clock::time_point last_update_time_;

    // 带决策的障碍物列表
    Obstacle::Obstacle::List obstacles_with_decision_;
    std::map<int, DecisionResult> decision_results_;

    // 决策历史
    std::map<int, ObstacleDecisionHistory> obstacle_histories_;

    // 状态机实例（每个决策类型对应一个状态对象）
    std::map<DecisionType, std::shared_ptr<DecisionState>> state_machines_;

    // 全局意图（融合所有障碍物决策后的最高优先级）
    DecisionType global_intention_ = DecisionType::IGNORE;

    // 调试信息
    mutable std::stringstream debug_info_;
    mutable std::mutex debug_mutex_;

    // 统计
    struct Statistics
    {
        int total_frames = 0;
        int total_obstacles_processed = 0;
        std::map<DecisionType, int> state_counts;
        std::map<TrafficScenario, int> scenario_counts;
    } statistics_;
};

} // namespace Decision