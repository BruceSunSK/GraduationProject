// piecewise_jerk_speed_smoother.h
#pragma once
#include <vector>
#include <array>
#include <cmath>
#include <algorithm>
#include <limits>
#include <iostream>
#include <memory>

#include "global_planning/path/utils.h"


namespace Smoother
{
class PiecewiseJerkSpeedSmoother
{
public:
    struct SpeedOptimizerParams
    {
        // 权重参数
        double weight_speed_deviation = 1.0;     // 速度偏差权重
        double weight_acceleration = 2.0;        // 加速度权重
        double weight_jerk = 5.0;                // 加加速度权重
        double weight_comfort = 0.1;             // 舒适性权重
        
        // 约束参数
        double max_speed = 4.0;                  // 最大速度(m/s)
        double min_speed = 0.0;                  // 最小速度(m/s)
        double max_acceleration = 2.0;           // 最大加速度(m/s²)
        double max_deceleration = -3.0;          // 最大减速度(m/s²)
        double max_jerk = 1.0;                   // 最大加加速度(m/s³)
        
        // 优化参数
        int max_iterations = 1000;               // 最大迭代次数
        double tolerance = 1e-6;                 // 收敛容忍度
    };
    
public:
    PiecewiseJerkSpeedSmoother() = default;
    PiecewiseJerkSpeedSmoother(const SpeedOptimizerParams & params) : params_(params) {}
    PiecewiseJerkSpeedSmoother(const PiecewiseJerkSpeedSmoother&) = delete;
    PiecewiseJerkSpeedSmoother(PiecewiseJerkSpeedSmoother&&) = delete;
    PiecewiseJerkSpeedSmoother & operator=(const PiecewiseJerkSpeedSmoother &) = delete;
    PiecewiseJerkSpeedSmoother & operator=(PiecewiseJerkSpeedSmoother&&) = delete;
    ~PiecewiseJerkSpeedSmoother() = default;
    
    void SetParams(const SpeedOptimizerParams & params) { params_ = params; }
    
    bool Solve(double init_s, double init_v, double init_a,
               const std::vector<double> & time_points,
               const std::vector<std::pair<double, double>> & st_lower_bound,
               const std::vector<std::pair<double, double>> & st_upper_bound,
               const std::vector<double> & s_reference,
               std::vector<Path::TrajectoryPoint> & speed_profile);
    
private:
    SpeedOptimizerParams params_;
    
    // 检查边界有效性
    bool ValidateBoundaries(const std::vector<double>& time_points,
                            const std::vector<std::pair<double, double>>& st_lower_bound,
                            const std::vector<std::pair<double, double>>& st_upper_bound) const;
    
    // 构建二次规划问题
    bool BuildQPProblem(const std::vector<double>& time_points,
                        const std::vector<std::pair<double, double>>& st_lower_bound,
                        const std::vector<std::pair<double, double>>& st_upper_bound,
                        const std::vector<double>& s_reference,
                        std::vector<double>& solution);
    
    // 解析解向量
    void ParseSolution(const std::vector<double>& solution,
                       const std::vector<double>& time_points,
                       std::vector<Path::TrajectoryPoint>& speed_profile);
};

} // namespace Smoother