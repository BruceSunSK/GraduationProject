// lattice_planner.cpp
#include "local_planning/planners/lattice_planner.h"


void LatticePlanner::InitParams(const LocalPlannerParamsInterface & params)
{
    LatticePlannerParams p = dynamic_cast<const LatticePlannerParams &>(params);
    params_ = p;
}

bool LatticePlanner::Plan(LocalPlannerResult & result, std::string & error_msg)
{
    result.Clear();
    if (!map_ || !reference_path_ || !vehicle_state_)
    {
        error_msg = "LatticePlanner: missing input data.";
        return false;
    }

    // 1. 获取车辆当前 Frenet 状态
    auto [proj, idx] = reference_path_->GetProjection({ vehicle_state_->pos.x, vehicle_state_->pos.y });
    FrenetState start;
    start.s = proj.s;
    start.l = proj.l;
    double cos_delta = std::cos(vehicle_state_->pos.theta - proj.theta);
    start.s_d = vehicle_state_->v * cos_delta / (1.0 - proj.kappa * start.l + 1e-6);
    double tan_delta = std::tan(vehicle_state_->pos.theta - proj.theta);
    start.l_d = tan_delta * (1.0 - proj.kappa * start.l);
    start.s_dd = 0.0;
    start.l_dd = 0.0;

    // 2. 生成采样终点
    double s_end_max = std::min(reference_path_->GetLength(), start.s + params_.sample_T_max * params_.max_speed);
    double s_end_min = start.s + params_.sample_T_min * 0.5;
    std::vector<SampleEndState> samples;
    for (double s_target = s_end_min; s_target <= s_end_max; s_target += params_.sample_s_step)
    {
        for (double l_target = -params_.sample_l_range; l_target <= params_.sample_l_range; l_target += params_.sample_l_step)
        {
            for (double T = params_.sample_T_min; T <= params_.sample_T_max; T += params_.sample_T_step)
            {
                samples.push_back({ s_target, l_target, T });
            }
        }
    }

    // 3. 生成候选轨迹
    auto trajectories = GenerateTrajectories(start, samples);
    if (trajectories.empty())
    {
        error_msg = "LatticePlanner: no valid trajectory found.";
        return false;
    }

    // 4. 选择最优轨迹
    auto best = std::min_element(trajectories.begin(), trajectories.end(),
        [](const Trajectory & a, const Trajectory & b) { return a.cost < b.cost; });
    result.trajectory = best->points;
    return true;
}

std::vector<LatticePlanner::Trajectory> LatticePlanner::GenerateTrajectories(
    const FrenetState & start, const std::vector<SampleEndState> & samples)
{
    std::vector<Trajectory> trajectories;
    for (const auto & sample : samples)
    {
        // 横向目标状态：终点 l_target，l_d=0，l_dd=0
        double l1 = sample.l_target;
        double l_d1 = 0.0;
        double l_dd1 = 0.0;

        // 纵向目标状态：终点 s_target，速度目标值，加速度 0
        double s1 = sample.s_target;
        double s_d1 = params_.target_speed;
        double s_dd1 = 0.0;

        double T = sample.T;

        // 计算多项式系数
        auto coeff_l = ComputeQuinticCoefficients(start.l, start.l_d, start.l_dd, l1, l_d1, l_dd1, T);
        auto coeff_s = ComputeQuarticCoefficients(start.s, start.s_d, start.s_dd, s1, s_d1, T);

        // 生成轨迹点
        Trajectory traj;
        traj.points.clear();
        int steps = static_cast<int>(T / 0.1) + 1;
        bool valid = true;
        for (int i = 0; i <= steps; ++i)
        {
            double t = i * 0.1;
            double s = 0.0, s_d = 0.0, s_dd = 0.0;
            for (int j = 0; j < 5; ++j)
            {
                s += coeff_s[j] * std::pow(t, j);
                if (j >= 1) s_d += j * coeff_s[j] * std::pow(t, j - 1);
                if (j >= 2) s_dd += j * (j - 1) * coeff_s[j] * std::pow(t, j - 2);
            }
            double l = 0.0, l_d = 0.0, l_dd = 0.0;
            for (int j = 0; j < 6; ++j)
            {
                l += coeff_l[j] * std::pow(t, j);
                if (j >= 1) l_d += j * coeff_l[j] * std::pow(t, j - 1);
                if (j >= 2) l_dd += j * (j - 1) * coeff_l[j] * std::pow(t, j - 2);
            }

            // 转换为笛卡尔坐标
            Path::PathNode ref_node = reference_path_->GetPathNode(s);
            Path::TrajectoryPoint pt;
            pt.t = t;
            pt.s = s;
            pt.l = l;
            pt.x = ref_node.x - l * std::sin(ref_node.theta);
            pt.y = ref_node.y + l * std::cos(ref_node.theta);
            double denom = 1.0 - ref_node.kappa * l;
            if (std::abs(denom) < 1e-6) denom = 1e-6;
            pt.theta = ref_node.theta + std::atan2(l_d, denom);
            pt.v = s_d;
            pt.a = s_dd;
            pt.kappa = (ref_node.kappa + l_dd) * denom / (std::pow(denom * denom + l_d * l_d, 1.5) + 1e-6);

            // 边界检查
            if (s < 0 || s > reference_path_->GetLength())
            {
                valid = false;
                break;
            }

            // 碰撞检测
            if (map_)
            {
                double mx = (pt.x - map_->origin_x) / map_->resolution;
                double my = (pt.y - map_->origin_y) / map_->resolution;
                if (map_->distance_map.IsInside(mx, my))
                {
                    double dist = map_->distance_map.GetDistance(mx, my);
                    if (dist < 0.5)
                    {
                        valid = false;
                        break;
                    }
                }
            }

            traj.points.push_back(pt);
        }

        if (valid)
        {
            traj.cost = EvaluateTrajectory(traj, start);
            trajectories.push_back(traj);
        }
    }
    return trajectories;
}

double LatticePlanner::EvaluateTrajectory(const Trajectory & traj, const FrenetState & start)
{
    double cost = 0.0;
    int n = traj.points.size();
    if (n == 0) return std::numeric_limits<double>::max();

    // 横向偏移代价
    for (const auto & pt : traj.points)
        cost += params_.weight_lateral_offset * std::abs(pt.l);

    // 速度偏差代价
    for (const auto & pt : traj.points)
    {
        double dv = pt.v - params_.target_speed;
        cost += params_.weight_speed_deviation * dv * dv;
    }

    // 横向加速度代价
    for (const auto & pt : traj.points)
    {
        double lat_acc = pt.v * pt.v * std::abs(pt.kappa);
        cost += params_.weight_lateral_accel * lat_acc * lat_acc;
    }

    // 纵向 jerk 代价
    for (size_t i = 1; i < n - 1; ++i)
    {
        double dt = traj.points[i + 1].t - traj.points[i - 1].t;
        if (dt > 1e-6)
        {
            double jerk = (traj.points[i + 1].a - traj.points[i - 1].a) / dt;
            cost += params_.weight_longitudinal_jerk * jerk * jerk;
        }
    }

    // 障碍物代价
    if (map_)
    {
        for (const auto & pt : traj.points)
        {
            double mx = (pt.x - map_->origin_x) / map_->resolution;
            double my = (pt.y - map_->origin_y) / map_->resolution;
            if (map_->distance_map.IsInside(mx, my))
            {
                double dist = map_->distance_map.GetDistance(mx, my);
                if (dist < 1.0)
                    cost += params_.weight_obstacle / (dist + 0.1);
            }
        }
    }

    // 时间代价
    double T = traj.points.back().t;
    cost += params_.weight_travel_time * T;

    return cost;
}

bool LatticePlanner::CheckCollision(const Trajectory & traj) const
{
    if (!map_) return false;
    for (const auto & pt : traj.points)
    {
        double mx = (pt.x - map_->origin_x) / map_->resolution;
        double my = (pt.y - map_->origin_y) / map_->resolution;
        if (map_->distance_map.IsInside(mx, my))
        {
            double dist = map_->distance_map.GetDistance(mx, my);
            if (dist < 0.5) return true;
        }
    }
    return false;
}

std::array<double, 6> LatticePlanner::ComputeQuinticCoefficients(
    double x0, double dx0, double ddx0,
    double x1, double dx1, double ddx1, double T)
{
    std::array<double, 6> c;
    double T2 = T * T;
    double T3 = T2 * T;
    double T4 = T3 * T;
    double T5 = T4 * T;

    c[0] = x0;
    c[1] = dx0;
    c[2] = ddx0 / 2.0;

    double a = x1 - x0 - dx0 * T - 0.5 * ddx0 * T2;
    double b = dx1 - dx0 - ddx0 * T;
    double d = ddx1 - ddx0;

    c[3] = (20.0 * a - 8.0 * b * T + d * T2) / (2.0 * T3);
    c[4] = (-30.0 * a + 14.0 * b * T - 2.0 * d * T2) / (2.0 * T4);
    c[5] = (12.0 * a - 6.0 * b * T + d * T2) / (2.0 * T5);
    return c;
}

std::array<double, 5> LatticePlanner::ComputeQuarticCoefficients(
    double x0, double dx0, double ddx0,
    double x1, double dx1, double T)
{
    std::array<double, 5> c;
    double T2 = T * T;
    double T3 = T2 * T;
    double T4 = T3 * T;

    c[0] = x0;
    c[1] = dx0;
    c[2] = ddx0 / 2.0;

    double a = x1 - x0 - dx0 * T - 0.5 * ddx0 * T2;
    double b = dx1 - dx0 - ddx0 * T;

    c[3] = (4.0 * a - b * T) / (T3);
    c[4] = (-3.0 * a + b * T) / (T4);
    return c;
}