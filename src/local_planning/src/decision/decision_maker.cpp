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
        state_history.pop_front();
    }
}

double ObstacleDecisionHistory::GetStateDuration() const
{
    auto now = std::chrono::steady_clock::now();
    return std::chrono::duration<double>(now - state_start_time).count();
}

double ObstacleDecisionHistory::GetStateConfidence() const
{
    // 指数加权置信度，近期权重高
    if (state_history.empty()) return 0.0;
    const double alpha = 0.7;
    double weight_sum = 0.0;
    double consistent_weight = 0.0;
    double weight = 1.0;
    for (auto it = state_history.rbegin(); it != state_history.rend(); ++it)
    {
        if (*it == current_state)
        {
            consistent_weight += weight;
        }
        weight_sum += weight;
        weight *= alpha;
    }
    return consistent_weight / weight_sum;
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
    state_machines_[DecisionType::IGNORE]   = DecisionStateFactory::CreateState(DecisionType::IGNORE);
    state_machines_[DecisionType::FOLLOW]   = DecisionStateFactory::CreateState(DecisionType::FOLLOW);
    state_machines_[DecisionType::OVERTAKE] = DecisionStateFactory::CreateState(DecisionType::OVERTAKE);
    state_machines_[DecisionType::YIELD]    = DecisionStateFactory::CreateState(DecisionType::YIELD);
    state_machines_[DecisionType::STOP]     = DecisionStateFactory::CreateState(DecisionType::STOP);
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

    // 1. 获取障碍物投影信息
    CalculateObstacleInfo(obstacle_copy);

    // 2. 获取历史记录
    int id = obstacle->GetId();
    bool is_new_obstacle = (obstacle_histories_.find(id) == obstacle_histories_.end());
    auto & history = obstacle_histories_[id];

    // 3. 创建上下文
    DecisionContext context = CreateDecisionContext(obstacle_copy);
    context.last_state = history.current_state;
    context.state_count = history.state_count;
    context.state_time = history.GetStateDuration();

    // 4. 识别当前场景
    TrafficScenario current_scenario = IdentifyScenario(obstacle_copy);
    bool scenario_changed = (!is_new_obstacle &&
        history.last_scenario != TrafficScenario::UNKNOWN &&
        history.last_scenario != current_scenario);
    history.last_scenario = current_scenario;

    // 5. 执行状态机（根据是否为新建/场景变化选择入口）
    DecisionType new_state = ExecuteStateMachine(obstacle_copy, context, is_new_obstacle, scenario_changed);

    // 6. 迟滞滤波
    DecisionType filtered_state = ApplyHysteresis(id, new_state);

    // 7. 更新最终状态历史
    history.UpdateState(filtered_state);

    result.decision_state = filtered_state;
    result.confidence = history.GetStateConfidence();

    // 8. 填充决策信息（简化版本，实际应由状态机内部决定）
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
    obstacle->CalculateProjection(reference_path_, ego_position_, ego_speed_, ego_acceleration_);
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

    // todo 获取代价地图值
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

    // todo
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
                                                DecisionContext & context,
                                                bool is_new_obstacle,
                                                bool scenario_changed)
{
    // 确定入口状态
    DecisionType entry_state;
    if (is_new_obstacle || scenario_changed)
    {
        // 根据当前场景选择初始状态
        TrafficScenario current_scenario = IdentifyScenario(obstacle);
        switch (current_scenario)
        {
        case TrafficScenario::SAME_DIRECTION:
            entry_state = DecisionType::FOLLOW;
            break;
        case TrafficScenario::OPPOSITE_DIRECTION:
        case TrafficScenario::CROSS_TRAFFIC:
            entry_state = DecisionType::YIELD;
            break;
        default:
            entry_state = DecisionType::IGNORE;
            break;
        }
    }
    else
    {
        // 使用上一帧的最终状态
        entry_state = context.last_state;
        // 若上一帧状态无效，也回退到初始状态
        if (entry_state == DecisionType::UNKNOWN)
        {
            TrafficScenario current_scenario = IdentifyScenario(obstacle);
            switch (current_scenario)
            {
            case TrafficScenario::SAME_DIRECTION:
                entry_state = DecisionType::FOLLOW;
                break;
            case TrafficScenario::OPPOSITE_DIRECTION:
            case TrafficScenario::CROSS_TRAFFIC:
                entry_state = DecisionType::YIELD;
                break;
            default:
                entry_state = DecisionType::IGNORE;
                break;
            }
        }
    }

    auto state_machine = state_machines_[entry_state];
    if (!state_machine)
        return DecisionType::IGNORE;

    return state_machine->Evaluate(context);
}

void DecisionMaker::ApplyDecisionToObstacle(Obstacle::Obstacle::Ptr obstacle,
                                            const DecisionResult & decision_result)
{
    obstacle->SetDecision(decision_result.decision_info);
}

DecisionType DecisionMaker::ApplyHysteresis(int obstacle_id, DecisionType new_state)
{
    auto & history = obstacle_histories_[obstacle_id];

    // 记录原始建议
    history.raw_state_history.push_back(new_state);
    if (history.raw_state_history.size() > 10)
        history.raw_state_history.pop_front();

    // 如果新状态与当前最终状态相同，直接返回
    if (new_state == history.current_state)
        return new_state;

    // 检查最近连续 need_count 帧原始建议是否全是 new_state
    int need_count = params_.decision_hysteresis_count;
    if (history.raw_state_history.size() < need_count)
        return history.current_state;

    bool all_same = true;
    for (size_t i = history.raw_state_history.size() - need_count;
        i < history.raw_state_history.size(); ++i)
    {
        if (history.raw_state_history[i] != new_state)
        {
            all_same = false;
            break;
        }
    }
    return all_same ? new_state : history.current_state;
}

TrafficScenario DecisionMaker::IdentifyScenario(const Obstacle::Obstacle::Ptr & obstacle) const
{
    // 如果障碍物不在自车前方，则视为未知场景
    if (!obstacle->GetProjection().is_ahead)
    {
        return TrafficScenario::UNKNOWN;
    }

    // 获取参考线在障碍物投影点处的切线方向
    double ref_theta = reference_path_->GetPathNode(obstacle->GetProjection().s).theta;

    // 获取障碍物朝向（假设为其运动方向）
    double obs_theta = tf2::getYaw(obstacle->GetPose().orientation);

    // 计算角度差并归一化到 [0, π]
    double angle_diff = std::abs(obs_theta - ref_theta);
    while (angle_diff > M_PI)
        angle_diff -= 2 * M_PI;
    angle_diff = std::abs(angle_diff);

    // 静止障碍物视为同向静态
    if (obstacle->GetSpeed() < 0.1)
        return TrafficScenario::SAME_DIRECTION;

    if (angle_diff < params_.angle_same_threshold)
        return TrafficScenario::SAME_DIRECTION;
    else if (angle_diff > params_.angle_opposite_threshold)
        return TrafficScenario::OPPOSITE_DIRECTION;
    else if (angle_diff > params_.angle_cross_low && angle_diff < params_.angle_cross_high)
        return TrafficScenario::CROSS_TRAFFIC;
    else
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
        // 初始边界（大范围，可由道路宽度限制替代）
        std::array<std::pair<double, double>, 3> point_bounds = {
            std::make_pair(-10.0, 10.0),
            std::make_pair(-10.0, 10.0),
            std::make_pair(-10.0, 10.0)
        };

        // 合并所有障碍物的边界影响
        for (const auto & obs : obstacles_with_decision_)
        {
            auto obs_bound = CalculateObstacleBoundary(obs, ref_pt);
            for (int i = 0; i < 3; ++i)
            {
                point_bounds[i].first  = std::max(point_bounds[i].first,  obs_bound[i].first);
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
    // 注意：l < 0 表示右侧，l > 0 表示左侧
    // bounds[i].first  = 下界（右侧边界，较小 l 值）
    // bounds[i].second = 上界（左侧边界，较大 l 值）
    switch (decision.type)
    {
    case DecisionType::OVERTAKE:
    {
        if (std::abs(s_diff) < 20.0)  // 在障碍物附近才施加约束
        {
            if (proj.l > 0)   // 障碍物在左侧，应从右侧超车（自车 l 需小于障碍物）
            {
                // 限制上界，使 l <= proj.l - margin
                double right_bound = proj.l - params_.overtake_lateral_margin;
                bounds[0].second = std::min(bounds[0].second, right_bound);
                bounds[1].second = std::min(bounds[1].second, right_bound);
                bounds[2].second = std::min(bounds[2].second, right_bound);
            }
            else // if (proj.l < 0)  // 障碍物在右侧，应从左侧超车（自车 l 需大于障碍物）
            {
                // 限制下界，使 l >= proj.l + margin
                double left_bound = proj.l + params_.overtake_lateral_margin;
                bounds[0].first = std::max(bounds[0].first, left_bound);
                bounds[1].first = std::max(bounds[1].first, left_bound);
                bounds[2].first = std::max(bounds[2].first, left_bound);
            }
        }
        break;
    }

    case DecisionType::FOLLOW:
    {
        if (std::abs(s_diff) < params_.min_follow_distance)
        {
            double limit = 1.5;  // 跟车横向限制
            bounds[0].first  = std::max(bounds[0].first,  -limit);
            bounds[0].second = std::min(bounds[0].second,  limit);
            bounds[1].first  = std::max(bounds[1].first,  -limit);
            bounds[1].second = std::min(bounds[1].second,  limit);
            bounds[2].first  = std::max(bounds[2].first,  -limit);
            bounds[2].second = std::min(bounds[2].second,  limit);
        }
        break;
    }

    case DecisionType::YIELD:
    case DecisionType::STOP:
    {
        if (s_diff > -params_.safe_distance && s_diff < proj.length)
        {
            double strict = 0.5;
            bounds[0].first  = std::max(bounds[0].first,  -strict);
            bounds[0].second = std::min(bounds[0].second,  strict);
            bounds[1].first  = std::max(bounds[1].first,  -strict);
            bounds[1].second = std::min(bounds[1].second,  strict);
            bounds[2].first  = std::max(bounds[2].first,  -strict);
            bounds[2].second = std::min(bounds[2].second,  strict);
        }
        break;
    }

    case DecisionType::IGNORE:
    default:
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
    sb.bounds.resize(num, std::make_pair(0.0, std::numeric_limits<double>::max()));

    // 生成时间点
    for (int i = 0; i < num; ++i)
    {
        double t = i * time_resolution;
        sb.time_points[i] = t;
    }

    // 如果没有障碍物，使用默认边界（下界为0，上界为无穷大，但实际规划中会由车辆动力学限制）
    if (obstacles_with_decision_.empty())
    {
        for (int i = 0; i < num; ++i)
        {
            sb.bounds[i].first = 0.0;
            sb.bounds[i].second = std::numeric_limits<double>::max();
        }
        return sb;
    }

    // 初始化每个时间点的边界为默认值
    for (int i = 0; i < num; ++i)
    {
        sb.bounds[i].first = 0.0;
        sb.bounds[i].second = std::numeric_limits<double>::max();
    }

    // 合并每个障碍物的 ST 边界
    for (const auto & obs : obstacles_with_decision_)
    {
        auto decision = obs->GetDecision();
        auto proj = obs->GetProjection();
        if (!proj.is_ahead) continue;  // 仅考虑前方障碍物

        for (int i = 0; i < num; ++i)
        {
            double t = sb.time_points[i];
            auto [rear_s, front_s] = CalculateSTBoundaryForObstacle(obs, t);

            switch (decision.type)
            {
            case DecisionType::FOLLOW:
                // 跟车：不能越过障碍物后方，即 s <= rear_s
                sb.bounds[i].second = std::min(sb.bounds[i].second, rear_s);
                break;
            case DecisionType::OVERTAKE:
                // 超车：必须超过障碍物前方，即 s >= front_s
                sb.bounds[i].first = std::max(sb.bounds[i].first, front_s);
                break;
            case DecisionType::YIELD:
            case DecisionType::STOP:
                // 让行/停车：不能进入障碍物区域，即 s <= rear_s
                sb.bounds[i].second = std::min(sb.bounds[i].second, rear_s);
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
        if (sb.bounds[i].first > sb.bounds[i].second)
        {
            // 若下界超过上界，取中点并收缩
            double mid = (sb.bounds[i].first + sb.bounds[i].second) / 2.0;
            sb.bounds[i].first = mid - 0.1;
            sb.bounds[i].second = mid + 0.1;
        }
        // 下界不能为负
        sb.bounds[i].first = std::max(0.0, sb.bounds[i].first);
        // 上界不小于下界
        sb.bounds[i].second = std::max(sb.bounds[i].second, sb.bounds[i].first);
    }
    return sb;
}

std::pair<double, double> DecisionMaker::CalculateSTBoundaryForObstacle(
    const Obstacle::Obstacle::Ptr & obstacle,
    double time) const
{
    auto proj = obstacle->GetProjection();
    // 障碍物预测位置（匀速假设）
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