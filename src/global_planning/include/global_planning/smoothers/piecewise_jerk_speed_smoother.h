// piecewise_jerk_speed_smoother.h
#pragma once
#include <vector>

#include <Eigen/Dense>
#include <Eigen/Sparse>
#include <OsqpEigen/OsqpEigen.h>

#include "global_planning/path/data_type.h"


namespace Smoother
{
class PiecewiseJerkSpeedSmoother
{
public:
    struct Weights
    {
        double w_speed_deviation;       // 速度偏差权重 (v - v_ref)
        double w_acceleration;          // 加速度权重 (s'')
        double w_jerk;                  // 加加速度权重 (s''')
        double w_lateral_acceleration;  // 横向加速度权重 (v * kappa)^2
    };

public:
    PiecewiseJerkSpeedSmoother() = delete;
    /**
     * @brief 构造函数，传入通用不变的参数
     * @param dt 时间步长（秒），假设均匀采样
     * @param weights 权重结构
     */
    PiecewiseJerkSpeedSmoother(double dt, const Weights & weights) : dt_(dt), weights_(weights) {}
    PiecewiseJerkSpeedSmoother(const PiecewiseJerkSpeedSmoother &) = delete;
    PiecewiseJerkSpeedSmoother(PiecewiseJerkSpeedSmoother &&) = delete;
    PiecewiseJerkSpeedSmoother & operator=(const PiecewiseJerkSpeedSmoother &) = delete;
    PiecewiseJerkSpeedSmoother & operator=(PiecewiseJerkSpeedSmoother &&) = delete;
    ~PiecewiseJerkSpeedSmoother() = default;

    /**
      * @brief 求解速度规划问题
      * @param s0 初始位置
      * @param v0 初始速度
      * @param a0 初始加速度
      * @param s_lower 位置下界数组（长度为N）
      * @param s_upper 位置上界数组（长度为N）
      * @param v_lower 速度下界数组（长度为N）
      * @param v_upper 速度上界数组（长度为N）
      * @param a_lower 加速度下界数组（长度为N）
      * @param a_upper 加速度上界数组（长度为N）
      * @param v_ref 参考速度数组（长度为N）
      * @param kappa_ref 参考曲率数组（长度为N），用于横向加速度代价
      * @param s_end 终点最大允许位置（约束 s_{N-1} <= s_end）
      * @param result 输出轨迹点，每个点包含 t, s, v, a（t 根据 dt 自动生成）
      * @return 是否求解成功
      */
    bool Solve(
        double s0, double v0, double a0,
        const std::vector<double> & s_lower,
        const std::vector<double> & s_upper,
        const std::vector<double> & v_lower,
        const std::vector<double> & v_upper,
        const std::vector<double> & a_lower,
        const std::vector<double> & a_upper,
        const std::vector<double> & v_ref,
        const std::vector<double> & kappa_ref,
        double s_end,
        std::vector<Path::TrajectoryPoint> & result) const;

private:
    double dt_;                     // 时间步长
    Weights weights_;               // 权重
};

} // namespace Smoother