// decision_maker.cpp
#include "local_planning/decision/decision_maker.h"
#include "local_planning/decision/decision_state_factory.h"


namespace Decision
{
// ==================== ObstacleDecisionHistory 实现 ====================
void ObstacleDecisionHistory::UpdateState(DecisionType new_state)
{
    previous_state = current_state;
    current_state = new_state;
    if (new_state == previous_state)
    {
        ++state_count;
    }
    else
    {
        state_count = 1;
        state_start_time = std::chrono::steady_clock::now();
    }
    state_history.push_back(new_state);
    if (state_history.size() > 10)
    {
        state_history.erase(state_history.begin());
    }
}

double ObstacleDecisionHistory::GetStateDuration() const
{
    auto now = std::chrono::steady_clock::now();
    return std::chrono::duration<double>(now - state_start_time).count();
}

double ObstacleDecisionHistory::GetStateConfidence() const
{
    if (state_history.empty()) return 0.0;
    int consistent = 0;
    for (auto s : state_history)
    {
        if (s == current_state) ++consistent;
    }
    return static_cast<double>(consistent) / state_history.size();
}

std::string ObstacleDecisionHistory::GetHistoryString() const
{
    std::ostringstream oss;
    oss << "Current: " << DecisionStateFactory::StateTypeToString(current_state)
        << " (count: " << state_count
        << ", conf: " << std::fixed << std::setprecision(2) << GetStateConfidence() << ")";
    return oss.str();
}

// ==================== DecisionResult 实现 ====================
std::string DecisionResult::ToString() const
{
    std::ostringstream oss;
    oss << "DecisionResult{state=" << DecisionStateFactory::StateTypeToString(decision_state)
        << ", conf=" << std::fixed << std::setprecision(2) << confidence
        << ", target_speed=" << decision_info.target_speed
        << ", safety_dist=" << decision_info.safety_distance
        << ", reasoning=" << reasoning << "}";
    return oss.str();
}

// ==================== DecisionMaker 实现 ====================
DecisionMaker::DecisionMaker()
{
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
    obstacle_histories_.clear();
    decision_results_.clear();
    obstacles_with_decision_.clear();
    ClearDebugInfo();
    AddDebugInfo("DecisionMaker initialized.");
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

    reference_path_ = reference_path;
    ego_position_ = ego_position;
    ego_speed_ = ego_speed;
    ego_acceleration_ = ego_acceleration;

    obstacles_with_decision_.clear();
    decision_results_.clear();

    AddDebugInfo("\n==========================================");
    AddDebugInfo("Decision Making Cycle Start");
    AddDebugInfo("Ego: s=" + std::to_string(ego_position.s) +
        ", l=" + std::to_string(ego_position.l) +
        ", v=" + std::to_string(ego_speed) + " m/s");
    AddDebugInfo("Obstacles: " + std::to_string(obstacles.size()));

    statistics_.total_frames++;
    statistics_.total_obstacles_processed += obstacles.size();

    global_intention_ = DecisionType::IGNORE;
    for (const auto & obstacle : obstacles)
    {
        DecisionResult result = ProcessObstacle(obstacle);
        if (result.IsValid())
        {
            decision_results_[obstacle->GetId()] = result;
            auto obstacle_with_decision = std::make_shared<Obstacle::Obstacle>(*obstacle);
            ApplyDecisionToObstacle(obstacle_with_decision, result);
            obstacles_with_decision_.push_back(obstacle_with_decision);

            statistics_.state_counts[result.decision_state]++;

            // 全局意图优先级
            if (result.decision_state == DecisionType::STOP)
                global_intention_ = DecisionType::STOP;
            else if (result.decision_state == DecisionType::YIELD && global_intention_ != DecisionType::STOP)
                global_intention_ = DecisionType::YIELD;
            else if (result.decision_state == DecisionType::OVERTAKE &&
                global_intention_ != DecisionType::STOP && global_intention_ != DecisionType::YIELD)
                global_intention_ = DecisionType::OVERTAKE;
            else if (result.decision_state == DecisionType::FOLLOW && global_intention_ == DecisionType::IGNORE)
                global_intention_ = DecisionType::FOLLOW;

            if (params_.enable_debug_output)
                AddObstacleDebugInfo(obstacle, result);
        }
    }

    AddDebugInfo("Global intention: " + DecisionStateFactory::StateTypeToString(global_intention_));
    AddDebugInfo("Decision Making Cycle Complete");
    AddDebugInfo("==========================================");
    last_update_time_ = std::chrono::steady_clock::now();
}

DecisionResult DecisionMaker::ProcessObstacle(const Obstacle::Obstacle::Ptr & obstacle)
{
    DecisionResult result;

    auto obstacle_copy = std::make_shared<Obstacle::Obstacle>(*obstacle);
    CalculateObstacleInfo(obstacle_copy);

    DecisionContext context = CreateDecisionContext(obstacle_copy);

    auto & history = obstacle_histories_[obstacle->GetId()];
    context.last_state = history.current_state;
    context.state_count = history.state_count;
    context.state_time = history.GetStateDuration();

    DecisionType new_state = ExecuteStateMachine(obstacle_copy, context);

    DecisionType filtered_state = ApplyDecisionFilter(obstacle->GetId(), new_state);
    filtered_state = ApplyHysteresis(obstacle->GetId(), filtered_state);

    history.UpdateState(filtered_state);

    result.decision_state = filtered_state;
    result.confidence = history.GetStateConfidence();

    // 填充决策信息（简化版本）
    switch (filtered_state)
    {
    case DecisionType::FOLLOW:
        result.decision_info.type = DecisionType::FOLLOW;
        result.decision_info.target_speed = std::max(0.0, obstacle_copy->GetSpeed() - 0.5);
        result.decision_info.safety_distance = params_.min_follow_distance;
        result.reasoning = "Following obstacle ahead";
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
        result.reasoning = "Yielding to crossing/oncoming obstacle";
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
        result.reasoning = "Obstacle not relevant";
        break;
    }
    result.decision_info.decision_confidence = result.confidence;
    return result;
}

void DecisionMaker::CalculateObstacleInfo(Obstacle::Obstacle::Ptr obstacle)
{
    obstacle->CalculateProjection(reference_path_, ego_position_);
    obstacle->UpdateCollisionTime(ego_speed_, ego_acceleration_);
}

DecisionContext DecisionMaker::CreateDecisionContext(const Obstacle::Obstacle::Ptr & obstacle) const
{
    DecisionContext ctx;
    ctx.ego_position = ego_position_;
    ctx.ego_speed = ego_speed_;
    ctx.ego_acceleration = ego_acceleration_;

    ctx.obstacle_id = obstacle->GetId();
    ctx.obstacle_type = obstacle->GetType();
    ctx.obstacle_speed = obstacle->GetSpeed();
    ctx.obstacle_dimension = obstacle->GetDimension();
    ctx.projection = obstacle->GetProjection();

    const auto & perc = obstacle->GetPerceptionObstacle();
    ctx.predicted_trajectory = perc.predicted_trajectory;

    // 获取曲率
    if (reference_path_)
    {
        Path::PathNode node = reference_path_->GetPathNode(ctx.projection.s);
        ctx.curvature = node.kappa;
    }
    else
    {
        ctx.curvature = 0.0;
    }

    // 获取代价地图值
    if (cost_map_)
    {
        double mx = (obstacle->GetPose().position.x - cost_map_->origin_x) / cost_map_->resolution;
        double my = (obstacle->GetPose().position.y - cost_map_->origin_y) / cost_map_->resolution;
        ctx.cost_value = cost_map_->distance_map.GetDistance(mx, my);
    }
    else
    {
        ctx.cost_value = 0.0;
    }

    ctx.lane_width = params_.default_lane_width;
    ctx.road_width = params_.default_lane_width * 2;

    ctx.last_state = DecisionType::UNKNOWN;
    ctx.state_count = 0;
    ctx.state_time = 0.0;
    ctx.timestamp = std::chrono::duration<double>(
        std::chrono::steady_clock::now().time_since_epoch()).count();

    return ctx;
}

DecisionType DecisionMaker::ExecuteStateMachine(const Obstacle::Obstacle::Ptr & obstacle,
    DecisionContext & context)
{
    TrafficScenario scenario = IdentifyScenario(obstacle);
    statistics_.scenario_counts[scenario]++;

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

    auto state_machine = state_machines_[initial_state];
    if (!state_machine)
        return DecisionType::IGNORE;

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
        return new_state;

    // 简单滤波：如果最近3次中有2次相同，则采用该状态
    int count = 0;
    int n = std::min(3, static_cast<int>(history.state_history.size()));
    for (int i = 0; i < n; ++i)
    {
        if (history.state_history[history.state_history.size() - 1 - i] == new_state)
            ++count;
    }
    return (count >= 2) ? new_state : history.current_state;
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
    double l = obstacle->GetProjection().l;
    // 同向交通：障碍物在自车前方或后方，横向偏移较小
    if (l >= -params_.same_direction_threshold && l <= params_.same_direction_threshold)
        return TrafficScenario::SAME_DIRECTION;
    // 对向交通：障碍物在相反方向
    if (l <= params_.opposite_direction_threshold)
        return TrafficScenario::OPPOSITE_DIRECTION;
    // 交叉交通：障碍物从侧面接近
    if (std::abs(l) <= params_.cross_traffic_threshold)
        return TrafficScenario::CROSS_TRAFFIC;
    return TrafficScenario::UNKNOWN;
}

// ==================== 边界生成 ====================

PathBoundary DecisionMaker::GeneratePathBoundary(const std::vector<Path::PathNode> & ref_points) const
{
    PathBoundary boundary;
    boundary.bounds.reserve(ref_points.size());
    boundary.s_coordinates.reserve(ref_points.size());

    for (const auto & ref_pt : ref_points)
    {
        std::array<std::pair<double, double>, 3> point_bounds = {
            std::make_pair(-10.0, 10.0),
            std::make_pair(-10.0, 10.0),
            std::make_pair(-10.0, 10.0)
        };

        for (const auto & obs : obstacles_with_decision_)
        {
            auto obs_bound = CalculateObstacleBoundary(obs, ref_pt);
            for (int i = 0; i < 3; ++i)
            {
                point_bounds[i].first = std::max(point_bounds[i].first, obs_bound[i].first);
                point_bounds[i].second = std::min(point_bounds[i].second, obs_bound[i].second);
            }
        }

        boundary.bounds.push_back(point_bounds);
        boundary.s_coordinates.push_back(ref_pt.s);
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
        if (std::abs(s_diff) < 20.0)
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
            double limit = 1.5;
            bounds[0].first = -limit;
            bounds[0].second = limit;
            bounds[1].first = -limit;
            bounds[1].second = limit;
            bounds[2].first = -limit;
            bounds[2].second = limit;
        }
        break;
    }

    case DecisionType::YIELD:
    case DecisionType::STOP:
    {
        // 让行或停车：在障碍物前方严格限制边界
        if (s_diff > -params_.safe_distance && s_diff < proj.length)
        {
            double strict = 0.5;
            bounds[0].first = -strict;
            bounds[0].second = strict;
            bounds[1].first = -strict;
            bounds[1].second = strict;
            bounds[2].first = -strict;
            bounds[2].second = strict;
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

SpeedBoundary DecisionMaker::GenerateSpeedBoundary(double planning_horizon,
    double time_resolution) const
{
    int num = static_cast<int>(planning_horizon / time_resolution) + 1;
    SpeedBoundary sb;
    sb.time_points.resize(num);
    sb.s_points.resize(num);
    sb.st_lower_bound.resize(num, { 0.0, std::numeric_limits<double>::max() });
    sb.st_upper_bound.resize(num, { std::numeric_limits<double>::lowest(), 0.0 });

    for (int i = 0; i < num; ++i)
    {
        double t = i * time_resolution;
        sb.time_points[i] = t;
        sb.s_points[i] = ego_position_.s + ego_speed_ * t;
    }

    // 如果没有障碍物，使用默认边界
    if (obstacles_with_decision_.empty())
    {
        for (int i = 0; i < num; ++i)
        {
            sb.st_lower_bound[i] = { sb.time_points[i], 0.0 };
            sb.st_upper_bound[i] = { sb.time_points[i], sb.s_points.back() };
        }
        return sb;
    }

    // 计算每个障碍物的ST边界
    for (const auto & obs : obstacles_with_decision_)
    {
        auto decision = obs->GetDecision();
        auto proj = obs->GetProjection();
        if (!proj.is_ahead) continue;

        for (int i = 0; i < num; ++i)
        {
            double t = sb.time_points[i];
            auto [rear_s, front_s] = CalculateSTBoundaryForObstacle(obs, t);

            // 根据决策类型合并边界
            switch (decision.type)
            {
            case DecisionType::FOLLOW:
                // 跟车：障碍物后方为下界
                sb.st_lower_bound[i].second = std::min(sb.st_lower_bound[i].second, rear_s);
                break;
            case DecisionType::OVERTAKE:
                // 超车：障碍物前方为上界
                sb.st_upper_bound[i].first = std::max(sb.st_upper_bound[i].first, front_s);
                break;
            case DecisionType::YIELD:
            case DecisionType::STOP:
                // 让行或停车：障碍物前方严格限制
                sb.st_upper_bound[i].second = std::min(sb.st_upper_bound[i].second, rear_s);
                break;
            case DecisionType::IGNORE:
            default:
                break;
            }
        }
    }

    // 确保边界合理性
    for (int i = 0; i < num; ++i)
    {
        // 下界不能超过上界
        if (sb.st_lower_bound[i].second > sb.st_upper_bound[i].second)
        {
            sb.st_lower_bound[i].second = sb.st_upper_bound[i].second - 0.1;
        }
        // 边界不能为负
        sb.st_lower_bound[i].second = std::max(0.0, sb.st_lower_bound[i].second);
        sb.st_upper_bound[i].second = std::max(0.0, sb.st_upper_bound[i].second);
    }
    return sb;
}

std::pair<double, double> DecisionMaker::CalculateSTBoundaryForObstacle(
    const Obstacle::Obstacle::Ptr & obstacle,
    double time) const
{
    auto proj = obstacle->GetProjection();
    // 障碍物预测位置
    double obstacle_s = proj.s + obstacle->GetSpeed() * time;
    double half_len = proj.length / 2.0;

    // 障碍物前后沿
    double rear = obstacle_s - half_len;
    double front = obstacle_s + half_len;

    // 添加安全距离
    double safety = obstacle->GetDecision().safety_distance;
    return { rear - safety, front + safety };
}

// ==================== 调试函数 ====================
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
    std::ostringstream oss;
    oss << "\nObstacle " << obstacle->GetId() << ":\n"
        << "  Type: " << static_cast<int>(obstacle->GetType()) << "\n"
        << "  s=" << std::fixed << std::setprecision(2) << proj.s
        << ", l=" << proj.l << "\n"
        << "  Speed: " << obstacle->GetSpeed() << " m/s (rel=" << proj.relative_speed << ")\n"
        << "  TTC: " << proj.time_to_collision << " s\n"
        << "  Scenario: " << ScenarioToString(scenario) << "\n"
        << "  Decision: " << result.ToString() << "\n"
        << "  History: " << history.GetHistoryString() << "\n";
    AddDebugInfo(oss.str());
}

std::string DecisionMaker::GetDebugInfo() const
{
    std::lock_guard<std::mutex> lock(debug_mutex_);
    return debug_info_.str();
}

std::string DecisionMaker::GetStatistics() const
{
    std::ostringstream oss;
    oss << "Decision Statistics:\n"
        << "  Frames: " << statistics_.total_frames << "\n"
        << "  Obstacles processed: " << statistics_.total_obstacles_processed << "\n"
        << "  State counts:\n";
    for (const auto & [state, cnt] : statistics_.state_counts)
        oss << "    " << DecisionStateFactory::StateTypeToString(state) << ": " << cnt << "\n";
    oss << "  Scenario counts:\n";
    for (const auto & [scenario, cnt] : statistics_.scenario_counts)
        oss << "    " << ScenarioToString(scenario) << ": " << cnt << "\n";
    return oss.str();
}

void DecisionMaker::ClearDebugInfo()
{
    std::lock_guard<std::mutex> lock(debug_mutex_);
    debug_info_.str("");
    debug_info_.clear();
}

} // namespace Decision