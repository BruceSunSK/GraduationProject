#include "control/controller/pid_controller.h"


PIDController::PIDController(double kp, double ki, double kd)
    : kp_(kp), ki_(ki), kd_(kd),
    prev_error_(0.0), integral_(0.0),
    output_min_(-std::numeric_limits<double>::max()),
    output_max_(std::numeric_limits<double>::max()),
    integral_min_(-std::numeric_limits<double>::max()),
    integral_max_(std::numeric_limits<double>::max()),
    first_run_(true)
{
}

double PIDController::compute(double setpoint, double measurement)
{
    std::chrono::steady_clock::time_point current_time = std::chrono::steady_clock::now();
    double dt = 0.0;

    if (!first_run_)
    {
        dt = std::chrono::duration_cast<std::chrono::duration<double>>(current_time - prev_time_).count();
    }
    else
    {
        first_run_ = false;
        dt = 0.01; // 初始时间步长
    }

    // 计算误差
    double error = setpoint - measurement;

    // 积分项
    integral_ += error * dt;

    // 限制积分项
    if (integral_ > integral_max_) integral_ = integral_max_;
    if (integral_ < integral_min_) integral_ = integral_min_;

    // 微分项
    double derivative = 0.0;
    if (dt > 0.0)
    {
        derivative = (error - prev_error_) / dt;
    }

    // 计算输出
    double output = kp_ * error + ki_ * integral_ + kd_ * derivative;

    // 限制输出
    if (output > output_max_) output = output_max_;
    if (output < output_min_) output = output_min_;

    // 保存状态
    prev_error_ = error;
    prev_time_ = current_time;

    return output;
}

void PIDController::reset()
{
    prev_error_ = 0.0;
    integral_ = 0.0;
    first_run_ = true;
}

void PIDController::setOutputLimits(double min, double max)
{
    output_min_ = min;
    output_max_ = max;
}

void PIDController::setIntegralLimits(double min, double max)
{
    integral_min_ = min;
    integral_max_ = max;
}