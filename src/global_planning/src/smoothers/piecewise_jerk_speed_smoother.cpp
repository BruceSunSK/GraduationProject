// piecewise_jerk_speed_smoother.cpp
#include "global_planning/smoothers/piecewise_jerk_speed_smoother.h"


namespace Smoother
{
bool PiecewiseJerkSpeedSmoother::Solve(double init_s, double init_v, double init_a,
                                        const std::vector<double> & time_points,
                                        const std::vector<std::pair<double, double>> & st_lower_bound,
                                        const std::vector<std::pair<double, double>> & st_upper_bound,
                                        const std::vector<double> & s_reference,
                                        std::vector<Path::TrajectoryPoint> & speed_profile)
{
    speed_profile.clear();

    // 1. 验证输入数据
    if (time_points.empty())
    {
        std::cerr << "PiecewiseJerkSpeedSmoother: time_points is empty." << std::endl;
        return false;
    }

    if (!ValidateBoundaries(time_points, st_lower_bound, st_upper_bound))
    {
        std::cerr << "PiecewiseJerkSpeedSmoother: boundary validation failed." << std::endl;
        return false;
    }

    // 2. 构建并求解QP问题
    std::vector<double> solution;
    if (!BuildQPProblem(time_points, st_lower_bound, st_upper_bound, s_reference, solution))
    {
        std::cerr << "PiecewiseJerkSpeedSmoother: QP problem building failed." << std::endl;
        return false;
    }

    // 3. 解析解向量
    ParseSolution(solution, time_points, speed_profile);

    // 4. 设置初始状态
    if (!speed_profile.empty())
    {
        speed_profile[0].s = init_s;
        speed_profile[0].v = init_v;
        speed_profile[0].a = init_a;
    }

    return true;
}

bool PiecewiseJerkSpeedSmoother::ValidateBoundaries(const std::vector<double> & time_points,
    const std::vector<std::pair<double, double>> & st_lower_bound,
    const std::vector<std::pair<double, double>> & st_upper_bound) const
{
    if (st_lower_bound.size() != time_points.size() ||
        st_upper_bound.size() != time_points.size())
    {
        std::cerr << "PiecewiseJerkSpeedSmoother: boundary size mismatch." << std::endl;
        return false;
    }

    // 检查边界合理性
    for (size_t i = 0; i < time_points.size(); ++i)
    {
        if (st_lower_bound[i].first > st_upper_bound[i].first ||
            st_lower_bound[i].second > st_upper_bound[i].second)
        {
            std::cerr << "PiecewiseJerkSpeedSmoother: invalid boundary at time "
                << time_points[i] << std::endl;
            return false;
        }
    }

    return true;
}

bool PiecewiseJerkSpeedSmoother::BuildQPProblem(const std::vector<double> & time_points,
    const std::vector<std::pair<double, double>> & st_lower_bound,
    const std::vector<std::pair<double, double>> & st_upper_bound,
    const std::vector<double> & s_reference,
    std::vector<double> & solution)
{
    // 这里实现了一个简化的速度规划器
    // 在实际应用中，应该使用真正的QP求解器（如OSQP、qpOASES等）

    size_t n = time_points.size();
    if (n < 2)
    {
        std::cerr << "PiecewiseJerkSpeedSmoother: not enough time points." << std::endl;
        return false;
    }

    // 简化的速度规划：使用梯形速度曲线
    double total_time = time_points.back() - time_points.front();
    double total_s_distance = s_reference.back() - s_reference.front();

    if (total_time <= 0 || total_s_distance <= 0)
    {
        std::cerr << "PiecewiseJerkSpeedSmoother: invalid time or distance." << std::endl;
        return false;
    }

    // 计算平均速度
    double avg_speed = total_s_distance / total_time;

    // 限制速度在允许范围内
    avg_speed = std::max(params_.min_speed, std::min(params_.max_speed, avg_speed));

    // 生成速度剖面（简化的梯形速度曲线）
    solution.resize(n);

    // 假设匀速运动
    for (size_t i = 0; i < n; ++i)
    {
        double t = time_points[i];
        double s = s_reference.front() + avg_speed * t;

        // 确保s在边界内
        if (i < st_lower_bound.size() && i < st_upper_bound.size())
        {
            s = std::max(st_lower_bound[i].second, std::min(st_upper_bound[i].second, s));
        }

        // 确保s在参考范围内
        if (!s_reference.empty())
        {
            s = std::max(s_reference.front(), std::min(s_reference.back(), s));
        }

        solution[i] = s;
    }

    return true;
}

void PiecewiseJerkSpeedSmoother::ParseSolution(const std::vector<double> & solution,
    const std::vector<double> & time_points,
    std::vector<Path::TrajectoryPoint> & speed_profile)
{
    speed_profile.clear();

    if (solution.size() != time_points.size())
    {
        std::cerr << "PiecewiseJerkSpeedSmoother: solution size mismatch." << std::endl;
        return;
    }

    for (size_t i = 0; i < solution.size(); ++i)
    {
        Path::TrajectoryPoint point;
        point.t = time_points[i];
        point.s = solution[i];

        // 计算速度（差分）
        if (i == 0)
        {
            point.v = 0.0;
        }
        else
        {
            double dt = time_points[i] - time_points[i - 1];
            if (dt > 0)
            {
                point.v = (solution[i] - solution[i - 1]) / dt;

                // 限制速度
                point.v = std::max(params_.min_speed, std::min(params_.max_speed, point.v));
            }
            else
            {
                point.v = 0.0;
            }
        }

        // 计算加速度（差分）
        if (i == 0)
        {
            point.a = 0.0;
        }
        else if (i == 1)
        {
            double dt = time_points[i] - time_points[i - 1];
            if (dt > 0)
            {
                point.a = (point.v - speed_profile[i - 1].v) / dt;
            }
            else
            {
                point.a = 0.0;
            }
        }
        else
        {
            double dt = time_points[i] - time_points[i - 1];
            if (dt > 0)
            {
                point.a = (point.v - speed_profile[i - 1].v) / dt;
            }
            else
            {
                point.a = 0.0;
            }
        }

        // 计算加加速度（差分）
        if (i < 2)
        {
            point.j = 0.0;
        }
        else
        {
            double dt = time_points[i] - time_points[i - 1];
            if (dt > 0)
            {
                point.j = (point.a - speed_profile[i - 1].a) / dt;
            }
            else
            {
                point.j = 0.0;
            }
        }

        // 限制加速度和加加速度
        point.a = std::max(params_.max_deceleration, std::min(params_.max_acceleration, point.a));
        point.j = std::max(-params_.max_jerk, std::min(params_.max_jerk, point.j));

        speed_profile.push_back(point);
    }
}

} // namespace Smoother