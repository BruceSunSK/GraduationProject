// decision_maker.cpp
#include "local_planning/decision/decision_maker.h"

#include "local_planning/decision/ignore_state.h"
#include "local_planning/decision/follow_state.h"
#include "local_planning/decision/overtake_state.h"
#include "local_planning/decision/yield_state.h"
#include "local_planning/decision/stop_state.h"

namespace Decision
{
// ObstacleDecisionHistory 实现
void ObstacleDecisionHistory::UpdateState(DecisionType new_state)
{
    previous_state = current_state;
    current_state = new_state;

    if (new_state == previous_state)
    {
        state_count++;
    }
    else
    {
        state_count = 1;
        state_start_time = std::chrono::steady_clock::now();
    }

    // 记录历史（保持最近10个状态）
    state_history.push_back(new_state);
    if (state_history.size() > 10)
    {
        state_history.erase(state_history.begin());
    }

    // 更新统计
    switch (new_state)
    {
    case DecisionType::IGNORE: ignore_count++; break;
    case DecisionType::FOLLOW: follow_count++; break;
    case DecisionType::OVERTAKE: overtake_count++; break;
    case DecisionType::YIELD: yield_count++; break;
    case DecisionType::STOP: stop_count++; break;
    default: break;
    }
}

double ObstacleDecisionHistory::GetStateDuration() const
{
    auto now = std::chrono::steady_clock::now();
    auto duration = std::chrono::duration<double>(now - state_start_time);
    return duration.count();
}

double ObstacleDecisionHistory::GetStateConfidence() const
{
    if (state_history.empty()) return 0.0;

    // 计算最近几个状态的一致性
    int consistent_count = 0;
    for (size_t i = 0; i < state_history.size(); ++i)
    {
        if (state_history[i] == current_state)
        {
            consistent_count++;
        }
    }

    return static_cast<double>(consistent_count) / state_history.size();
}

std::string ObstacleDecisionHistory::GetHistoryString() const
{
    std::stringstream ss;
    ss << "Current: " << DecisionStateFactory::StateTypeToString(current_state)
        << " (count: " << state_count
        << ", confidence: " << std::fixed << std::setprecision(2) << GetStateConfidence() << ")";
    return ss.str();
}

// DecisionResult 实现
std::string DecisionResult::ToString() const
{
    std::stringstream ss;
    ss << "DecisionResult{state=" << DecisionStateFactory::StateTypeToString(decision_state)
        << ", confidence=" << std::fixed << std::setprecision(2) << confidence
        << ", target_speed=" << decision_info.target_speed
        << ", safety_distance=" << decision_info.safety_distance
        << ", reasoning=" << reasoning << "}";
    return ss.str();
}

// DecisionMaker 实现
DecisionMaker::DecisionMaker()
{
    // 创建所有状态机实例
    state_machines_[DecisionType::IGNORE] = DecisionStateFactory::CreateState(DecisionType::IGNORE);
    state_machines_[DecisionType::FOLLOW] = DecisionStateFactory::CreateState(DecisionType::FOLLOW);
    state_machines_[DecisionType::OVERTAKE] = DecisionStateFactory::CreateState(DecisionType::OVERTAKE);
    state_machines_[DecisionType::YIELD] = DecisionStateFactory::CreateState(DecisionType::YIELD);
    state_machines_[DecisionType::STOP] = DecisionStateFactory::CreateState(DecisionType::STOP);

    last_update_time_ = std::chrono::steady_clock::now();
}

void DecisionMaker::Initialize(const DecisionMakerParams & params)
{
    params_ = params;
    is_initialized_ = true;

    // 重置状态
    obstacle_histories_.clear();
    decision_results_.clear();
    obstacles_with_decision_.clear();

    ClearDebugInfo();

    AddDebugInfo("DecisionMaker initialized with parameters:");
    AddDebugInfo("  safe_distance: " + std::to_string(params.safe_distance));
    AddDebugInfo("  min_follow_distance: " + std::to_string(params.min_follow_distance));
    AddDebugInfo("  overtake_lateral_margin: " + std::to_string(params.overtake_lateral_margin));
    AddDebugInfo("  safe_time_headway: " + std::to_string(params.safe_time_headway));
}

void DecisionMaker::UpdateAndDecide(const Obstacle::Obstacle::List & obstacles,
    const Path::ReferencePath::Ptr & reference_path,
    const Path::PathNode & ego_position,
    double ego_speed,
    double ego_acceleration)
{
    if (!is_initialized_)
    {
        AddDebugInfo("DecisionMaker not initialized!");
        return;
    }

    std::lock_guard<std::mutex> lock(debug_mutex_);

    // 更新当前帧数据
    reference_path_ = reference_path;
    ego_position_ = ego_position;
    ego_speed_ = ego_speed;
    ego_acceleration_ = ego_acceleration;

    // 清空上一帧的结果
    obstacles_with_decision_.clear();
    decision_results_.clear();

    AddDebugInfo("\n==========================================");
    AddDebugInfo("Decision Making Cycle Start");
    AddDebugInfo("==========================================");
    AddDebugInfo("Ego State: s=" + std::to_string(ego_position.s) +
        ", l=" + std::to_string(ego_position.l) +
        ", speed=" + std::to_string(ego_speed) + "m/s");
    AddDebugInfo("Obstacles to process: " + std::to_string(obstacles.size()));

    statistics_.total_frames++;
    statistics_.total_obstacles_processed += obstacles.size();

    // 处理每个障碍物
    for (const auto & obstacle : obstacles)
    {
        DecisionResult result = ProcessObstacle(obstacle);

        if (result.IsValid())
        {
            decision_results_[obstacle->GetId()] = result;

            // 创建带决策的障碍物副本
            auto obstacle_with_decision = std::make_shared<Obstacle::Obstacle>(*obstacle);
            ApplyDecisionToObstacle(obstacle_with_decision, result);
            obstacles_with_decision_.push_back(obstacle_with_decision);

            // 更新统计
            statistics_.state_counts[result.decision_state]++;

            // 调试输出
            if (params_.enable_debug_output)
            {
                AddObstacleDebugInfo(obstacle, result);
            }
        }
    }

    AddDebugInfo("Decision Making Cycle Complete");
    AddDebugInfo("==========================================");

    last_update_time_ = std::chrono::steady_clock::now();
}

DecisionResult DecisionMaker::ProcessObstacle(const Obstacle::Obstacle::Ptr & obstacle)
{
    DecisionResult result;

    // 1. 计算障碍物信息
    auto obstacle_copy = std::make_shared<Obstacle::Obstacle>(*obstacle);
    CalculateObstacleInfo(obstacle_copy);

    // 2. 创建决策上下文
    DecisionContext context = CreateDecisionContext(obstacle_copy);

    // 3. 获取决策历史
    auto & history = obstacle_histories_[obstacle->GetId()];
    context.last_state = history.current_state;
    context.state_count = history.state_count;

    // 4. 执行状态机
    DecisionType new_state = ExecuteStateMachine(obstacle_copy, context);

    // 5. 应用决策滤波和迟滞
    DecisionType filtered_state = ApplyDecisionFilter(obstacle->GetId(), new_state);
    filtered_state = ApplyHysteresis(obstacle->GetId(), filtered_state);

    // 6. 更新历史
    history.UpdateState(filtered_state);

    // 7. 构建决策结果
    result.decision_state = filtered_state;
    result.confidence = history.GetStateConfidence();

    // 8. 转换为障碍物决策信息
    switch (filtered_state)
    {
    case DecisionType::FOLLOW:
        result.decision_info.type = DecisionType::FOLLOW;
        result.decision_info.target_speed = std::max(0.0, obstacle_copy->GetSpeed() - 0.5);
        result.decision_info.safety_distance = params_.min_follow_distance;
        result.reasoning = "Following obstacle in same lane";
        break;

    case DecisionType::OVERTAKE:
        result.decision_info.type = DecisionType::OVERTAKE;
        result.decision_info.target_speed = ego_speed_ + params_.min_overtake_speed_gain;
        result.decision_info.safety_distance = params_.overtake_lateral_margin;
        result.reasoning = "Overtaking slower obstacle";
        break;

    case DecisionType::YIELD:
        result.decision_info.type = DecisionType::YIELD;
        result.decision_info.target_speed = std::max(0.0, ego_speed_ - 2.0);
        result.decision_info.safety_distance = params_.safe_distance * 1.5;
        result.reasoning = "Yielding to crossing or oncoming obstacle";
        break;

    case DecisionType::STOP:
        result.decision_info.type = DecisionType::STOP;
        result.decision_info.target_speed = 0.0;
        result.decision_info.safety_distance = params_.safe_distance * 2.0;
        result.reasoning = "Emergency stop required";
        break;

    case DecisionType::IGNORE:
    default:
        result.decision_info.type = DecisionType::UNKNOWN;
        result.decision_info.target_speed = ego_speed_;
        result.decision_info.safety_distance = params_.safe_distance;
        result.reasoning = "Obstacle not relevant for planning";
        break;
    }

    result.decision_info.decision_confidence = result.confidence;

    return result;
}

void DecisionMaker::CalculateObstacleInfo(Obstacle::Obstacle::Ptr obstacle)
{
    // 计算投影信息
    obstacle->CalculateProjection(reference_path_, ego_position_);

    // 更新碰撞时间
    obstacle->UpdateCollisionTime(ego_speed_, ego_acceleration_);
}

DecisionContext DecisionMaker::CreateDecisionContext(const Obstacle::Obstacle::Ptr & obstacle) const
{
    DecisionContext context;

    context.ego_position = ego_position_;
    context.ego_speed = ego_speed_;
    context.ego_acceleration = ego_acceleration_;

    context.obstacle_type = obstacle->GetType();
    context.obstacle_speed = obstacle->GetSpeed();
    context.obstacle_dimension = obstacle->GetDimension();
    context.projection = obstacle->GetProjection();

    context.lane_width = params_.default_lane_width;
    context.road_width = params_.default_lane_width * 2;

    // 时间戳
    context.timestamp = std::chrono::duration<double>(
        std::chrono::steady_clock::now().time_since_epoch()).count();

    return context;
}

DecisionType DecisionMaker::ExecuteStateMachine(const Obstacle::Obstacle::Ptr & obstacle,
    DecisionContext & context)
{
    // 识别场景
    TrafficScenario scenario = IdentifyScenario(obstacle);
    statistics_.scenario_counts[scenario]++;

    // 根据场景选择初始状态机
    DecisionType initial_state = DecisionType::IGNORE;

    switch (scenario)
    {
    case TrafficScenario::SAME_DIRECTION:
        initial_state = DecisionType::FOLLOW;
        break;
    case TrafficScenario::OPPOSITE_DIRECTION:
        initial_state = DecisionType::YIELD;
        break;
    case TrafficScenario::CROSS_TRAFFIC:
        initial_state = DecisionType::YIELD;
        break;
    default:
        initial_state = DecisionType::IGNORE;
        break;
    }

    // 获取状态机实例
    auto state_machine = state_machines_[initial_state];
    if (!state_machine)
    {
        return DecisionType::IGNORE;
    }

    // 执行状态评估
    return state_machine->Evaluate(context);
}

void DecisionMaker::ApplyDecisionToObstacle(Obstacle::Obstacle::Ptr obstacle,
    const DecisionResult & decision_result)
{
    obstacle->SetDecision(decision_result.decision_info);
}

DecisionType DecisionMaker::ApplyDecisionFilter(int obstacle_id, DecisionType new_state)
{
    auto & history = obstacle_histories_[obstacle_id];

    // 如果历史为空，直接返回新状态
    if (history.state_history.empty())
    {
        return new_state;
    }

    // 简单滤波：如果最近3次中有2次相同，则采用该状态
    int count = 0;
    int history_size = static_cast<int>(history.state_history.size());
    int check_count = std::min(3, history_size);

    for (int i = 0; i < check_count; ++i)
    {
        if (history.state_history[history_size - 1 - i] == new_state)
        {
            count++;
        }
    }

    if (count >= 2)
    {
        return new_state;
    }

    // 否则保持原状态
    return history.current_state;
}

DecisionType DecisionMaker::ApplyHysteresis(int obstacle_id, DecisionType new_state)
{
    auto & history = obstacle_histories_[obstacle_id];

    // 如果状态改变，需要满足迟滞计数
    if (new_state != history.current_state)
    {
        // 检查是否连续多次建议相同的新状态
        if (history.state_count >= params_.decision_hysteresis_count)
        {
            return new_state;
        }
        else
        {
            return history.current_state;  // 保持原状态
        }
    }

    return new_state;
}

TrafficScenario DecisionMaker::IdentifyScenario(const Obstacle::Obstacle::Ptr & obstacle) const
{
    auto proj = obstacle->GetProjection();
    double lateral_offset = proj.l;

    // 同向交通：障碍物在自车前方或后方，横向偏移较小
    if (lateral_offset >= -params_.same_direction_threshold &&
        lateral_offset <= params_.same_direction_threshold)
    {
        return TrafficScenario::SAME_DIRECTION;
    }

    // 对向交通：障碍物在相反方向
    if (lateral_offset <= params_.opposite_direction_threshold)
    {
        return TrafficScenario::OPPOSITE_DIRECTION;
    }

    // 交叉交通：障碍物从侧面接近
    if (std::abs(lateral_offset) <= params_.cross_traffic_threshold)
    {
        return TrafficScenario::CROSS_TRAFFIC;
    }

    return TrafficScenario::UNKNOWN;
}

std::string DecisionMaker::GetScenarioString(TrafficScenario scenario) const
{
    switch (scenario)
    {
    case TrafficScenario::SAME_DIRECTION: return "SAME_DIRECTION";
    case TrafficScenario::OPPOSITE_DIRECTION: return "OPPOSITE_DIRECTION";
    case TrafficScenario::CROSS_TRAFFIC: return "CROSS_TRAFFIC";
    default: return "UNKNOWN";
    }
}

bool DecisionMaker::GetObstacleDecision(int obstacle_id, DecisionResult & result) const
{
    auto it = decision_results_.find(obstacle_id);
    if (it != decision_results_.end())
    {
        result = it->second;
        return true;
    }
    return false;
}

DecisionType DecisionMaker::GetObstacleDecisionState(int obstacle_id) const
{
    auto it = decision_results_.find(obstacle_id);
    if (it != decision_results_.end())
    {
        return it->second.decision_state;
    }
    return DecisionType::UNKNOWN;
}

std::string DecisionMaker::GetDebugInfo() const
{
    std::lock_guard<std::mutex> lock(debug_mutex_);
    return debug_info_.str();
}

std::string DecisionMaker::GetStatistics() const
{
    std::stringstream ss;
    ss << "Decision Statistics:\n";
    ss << "  Total frames processed: " << statistics_.total_frames << "\n";
    ss << "  Total obstacles processed: " << statistics_.total_obstacles_processed << "\n";
    ss << "  Decision state distribution:\n";

    for (const auto & [state, count] : statistics_.state_counts)
    {
        ss << "    " << DecisionStateFactory::StateTypeToString(state)
            << ": " << count << "\n";
    }

    ss << "  Scenario distribution:\n";
    for (const auto & [scenario, count] : statistics_.scenario_counts)
    {
        ss << "    " << GetScenarioString(scenario)
            << ": " << count << "\n";
    }

    return ss.str();
}

void DecisionMaker::ClearDebugInfo()
{
    std::lock_guard<std::mutex> lock(debug_mutex_);
    debug_info_.str("");
    debug_info_.clear();
}

void DecisionMaker::AddDebugInfo(const std::string & info)
{
    if (params_.enable_debug_output)
    {
        std::lock_guard<std::mutex> lock(debug_mutex_);
        debug_info_ << info << "\n";
    }
}

void DecisionMaker::AddObstacleDebugInfo(const Obstacle::Obstacle::Ptr & obstacle,
    const DecisionResult & result)
{
    auto proj = obstacle->GetProjection();
    auto & history = obstacle_histories_[obstacle->GetId()];
    TrafficScenario scenario = IdentifyScenario(obstacle);

    std::stringstream ss;
    ss << "\nObstacle " << obstacle->GetId() << ":\n";
    ss << "  Type: " << static_cast<int>(obstacle->GetType()) << "\n";
    ss << "  Position: s=" << std::fixed << std::setprecision(2) << proj.s
        << ", l=" << proj.l << "\n";
    ss << "  Speed: " << obstacle->GetSpeed() << "m/s (relative: "
        << proj.relative_speed << "m/s)\n";
    ss << "  TTC: " << proj.time_to_collision << "s\n";
    ss << "  Scenario: " << GetScenarioString(scenario) << "\n";
    ss << "  Decision: " << result.ToString() << "\n";
    ss << "  History: " << history.GetHistoryString() << "\n";

    AddDebugInfo(ss.str());
}

// 边界生成实现
DecisionMaker::PathBoundary DecisionMaker::GeneratePathBoundary(
    const std::vector<Path::PathNode> & ref_points) const
{
    PathBoundary boundary;
    boundary.bounds.reserve(ref_points.size());
    boundary.s_coordinates.reserve(ref_points.size());

    for (const auto & ref_point : ref_points)
    {
        std::array<std::pair<double, double>, 3> point_bounds = {
            std::make_pair(-10.0, 10.0),  // c0 bounds
            std::make_pair(-10.0, 10.0),  // c1 bounds  
            std::make_pair(-10.0, 10.0)   // c2 bounds
        };

        // 合并所有障碍物的边界影响
        for (const auto & obstacle : obstacles_with_decision_)
        {
            auto obstacle_bound = CalculateObstacleBoundary(obstacle, ref_point);
            for (int i = 0; i < 3; ++i)
            {
                // 取最严格的边界（交集）
                point_bounds[i].first = std::max(point_bounds[i].first, obstacle_bound[i].first);
                point_bounds[i].second = std::min(point_bounds[i].second, obstacle_bound[i].second);
            }
        }

        boundary.bounds.push_back(point_bounds);
        boundary.s_coordinates.push_back(ref_point.s);
    }

    return boundary;
}

std::array<std::pair<double, double>, 3> DecisionMaker::CalculateObstacleBoundary(
    const Obstacle::Obstacle::Ptr & obstacle,
    const Path::PathNode & ref_point) const
{
    std::array<std::pair<double, double>, 3> bounds = {
        std::make_pair(-10.0, 10.0),
        std::make_pair(-10.0, 10.0),
        std::make_pair(-10.0, 10.0)
    };

    auto proj = obstacle->GetProjection();
    auto decision = obstacle->GetDecision();
    double s_diff = ref_point.s - proj.s;

    // 根据决策类型和距离设置边界
    switch (decision.type)
    {
    case DecisionType::OVERTAKE:
    {
        // 超车：在障碍物附近限制一侧的边界
        if (std::abs(s_diff) < 15.0)  // 在障碍物影响范围内
        {
            if (proj.l > 0)  // 障碍物在右侧，左侧超车
            {
                double left_bound = proj.l + params_.overtake_lateral_margin;
                bounds[0].first = left_bound;
                bounds[1].first = left_bound;
                bounds[2].first = left_bound;
            }
            else  // 障碍物在左侧，右侧超车
            {
                double right_bound = proj.l - params_.overtake_lateral_margin;
                bounds[0].second = right_bound;
                bounds[1].second = right_bound;
                bounds[2].second = right_bound;
            }
        }
        break;
    }

    case DecisionType::FOLLOW:
    {
        // 跟车：限制横向偏移范围
        if (std::abs(s_diff) < params_.min_follow_distance)
        {
            double lateral_limit = 1.0;  // 跟车时的横向限制
            bounds[0].first = -lateral_limit;
            bounds[0].second = lateral_limit;
            bounds[1].first = -lateral_limit;
            bounds[1].second = lateral_limit;
            bounds[2].first = -lateral_limit;
            bounds[2].second = lateral_limit;
        }
        break;
    }

    case DecisionType::YIELD:
    case DecisionType::STOP:
    {
        // 让行或停车：在障碍物前方严格限制边界
        if (s_diff > -params_.safe_distance && s_diff < proj.length)
        {
            double strict_limit = 0.5;  // 严格限制
            bounds[0].first = -strict_limit;
            bounds[0].second = strict_limit;
            bounds[1].first = -strict_limit;
            bounds[1].second = strict_limit;
            bounds[2].first = -strict_limit;
            bounds[2].second = strict_limit;
        }
        break;
    }

    case DecisionType::IGNORE:
    default:
        // 忽略或未知决策，不添加额外限制
        break;
    }

    return bounds;
}

DecisionMaker::SpeedBoundary DecisionMaker::GenerateSpeedBoundary(
    double planning_horizon, double time_resolution) const
{
    SpeedBoundary boundary;

    int num_points = static_cast<int>(planning_horizon / time_resolution) + 1;
    boundary.time_points.resize(num_points);
    boundary.s_points.resize(num_points);
    boundary.st_lower_bound.resize(num_points, { 0.0, std::numeric_limits<double>::max() });
    boundary.st_upper_bound.resize(num_points, { std::numeric_limits<double>::lowest(), 0.0 });

    // 生成时间点
    for (int i = 0; i < num_points; ++i)
    {
        double t = i * time_resolution;
        boundary.time_points[i] = t;
        boundary.s_points[i] = ego_position_.s + ego_speed_ * t;  // 简单的线性预测
    }

    // 如果没有障碍物，使用默认边界
    if (obstacles_with_decision_.empty())
    {
        for (int i = 0; i < num_points; ++i)
        {
            double t = boundary.time_points[i];
            double s_max = ego_position_.s + ego_speed_ * planning_horizon;  // 最大可能s
            boundary.st_lower_bound[i] = { 0.0, 0.0 };
            boundary.st_upper_bound[i] = { 0.0, s_max };
        }
        return boundary;
    }

    // 计算每个障碍物的ST边界
    for (const auto & obstacle : obstacles_with_decision_)
    {
        auto decision = obstacle->GetDecision();
        auto proj = obstacle->GetProjection();

        if (!proj.is_ahead) continue;

        for (int i = 0; i < num_points; ++i)
        {
            double t = boundary.time_points[i];
            auto st_bound = CalculateSTBoundaryForObstacle(obstacle, t, ego_speed_);

            // 根据决策类型合并边界
            switch (decision.type)
            {
            case DecisionType::FOLLOW:
                // 跟车：障碍物后方为下界
                boundary.st_lower_bound[i].second = std::min(
                    boundary.st_lower_bound[i].second, st_bound.first);
                break;

            case DecisionType::OVERTAKE:
                // 超车：障碍物前方为上界
                boundary.st_upper_bound[i].first = std::max(
                    boundary.st_upper_bound[i].first, st_bound.second);
                break;

            case DecisionType::YIELD:
            case DecisionType::STOP:
                // 让行或停车：障碍物前方严格限制
                boundary.st_upper_bound[i].second = std::min(
                    boundary.st_upper_bound[i].second, st_bound.first);
                break;

            case DecisionType::IGNORE:
            default:
                break;
            }
        }
    }

    // 确保边界合理性
    for (int i = 0; i < num_points; ++i)
    {
        // 下界不能超过上界
        if (boundary.st_lower_bound[i].second > boundary.st_upper_bound[i].second)
        {
            boundary.st_lower_bound[i].second = boundary.st_upper_bound[i].second - 0.1;
        }

        // 边界不能为负
        boundary.st_lower_bound[i].second = std::max(0.0, boundary.st_lower_bound[i].second);
        boundary.st_upper_bound[i].second = std::max(0.0, boundary.st_upper_bound[i].second);
    }

    return boundary;
}

std::pair<double, double> DecisionMaker::CalculateSTBoundaryForObstacle(
    const Obstacle::Obstacle::Ptr & obstacle,
    double time, double ego_speed) const
{
    auto proj = obstacle->GetProjection();

    // 障碍物预测位置
    double obstacle_s = proj.s + obstacle->GetSpeed() * time;
    double obstacle_length = proj.length;

    // 障碍物前后沿
    double rear_s = obstacle_s - obstacle_length / 2.0;
    double front_s = obstacle_s + obstacle_length / 2.0;

    // 添加安全距离
    double safety_margin = obstacle->GetDecision().safety_distance;
    rear_s -= safety_margin;
    front_s += safety_margin;

    return { rear_s, front_s };
}

} // namespace Decision