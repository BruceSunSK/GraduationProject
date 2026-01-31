#pragma once
#include <chrono>
#include <algorithm>


class PIDController
{
public:
    PIDController(double kp, double ki, double kd);

    // 计算PID输出
    double compute(double setpoint, double measurement);

    // 重置积分项
    void reset();

    // 设置输出限制
    void setOutputLimits(double min, double max);

    // 设置积分项限制
    void setIntegralLimits(double min, double max);

private:
    // PID参数
    double kp_;
    double ki_;
    double kd_;

    // 状态变量
    double prev_error_;
    double integral_;

    // 限制
    double output_min_;
    double output_max_;
    double integral_min_;
    double integral_max_;

    // 时间相关
    std::chrono::steady_clock::time_point prev_time_;
    bool first_run_;
};
