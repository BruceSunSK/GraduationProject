#pragma once

#include <cmath>
#include <random>
#include <chrono>
#include <memory>

// 车辆状态结构体
struct VehicleState
{
    double x;           // 位置 x (m)
    double y;           // 位置 y (m)
    double theta;       // 朝向 (rad)
    double v;           // 线速度 (m/s)
    double w;           // 角速度 (rad/s)
    std::chrono::steady_clock::time_point timestamp;

    VehicleState() : x(0.0), y(0.0), theta(0.0), v(0.0), w(0.0),
        timestamp(std::chrono::steady_clock::now())
    {
    }

    // 重置状态
    void reset()
    {
        x = y = theta = v = w = 0.0;
        timestamp = std::chrono::steady_clock::now();
    }
};

class DifferentialModel
{
public:
    DifferentialModel();

    // 初始化模型参数
    void initialize(double max_linear_vel, double max_angular_vel);

    // 设置初始状态
    void setInitialState(double x, double y, double theta);

    // 设置噪声参数
    void setNoiseParameters(double linear_noise_stddev = 0.02,
        double angular_noise_stddev = 0.05,
        double position_noise_stddev = 0.005,
        double velocity_noise_stddev = 0.01);

    // 更新车辆状态
    // add_noise: true表示使用加噪模型，false表示使用理想模型
    VehicleState update(double linear_vel, double angular_vel,
        double dt, bool add_noise = false);

    // 重置车辆状态
    void reset();

    // 获取当前状态
    VehicleState getCurrentState() const;

private:
    // 计算理想运动更新
    VehicleState updateIdeal(double linear_vel, double angular_vel, double dt);

    // 计算带噪声运动更新
    VehicleState updateNoisy(double linear_vel, double angular_vel, double dt);

    // 添加控制噪声
    void addControlNoise(double & linear_vel, double & angular_vel);

    // 添加过程噪声
    void addProcessNoise(VehicleState & state, double dt);

    // 添加测量噪声
    void addMeasurementNoise(VehicleState & state);

    // 限制控制输入
    void clampVelocity(double & linear_vel, double & angular_vel);

    // 归一化角度到[-π, π]
    void normalizeAngle(double & angle);

    // 车辆参数
    double max_linear_vel_;
    double max_angular_vel_;

    // 车辆状态
    VehicleState current_state_;
    VehicleState initial_state_;

    // 噪声参数
    double linear_noise_stddev_;
    double angular_noise_stddev_;
    double position_noise_stddev_;
    double velocity_noise_stddev_;

    // 随机数生成器
    std::default_random_engine random_generator_;
    std::normal_distribution<double> linear_noise_dist_;
    std::normal_distribution<double> angular_noise_dist_;
    std::normal_distribution<double> position_noise_dist_;
    std::normal_distribution<double> velocity_noise_dist_;

    // 用于噪声生成的状态
    struct
    {
        double linear_bias = 0.0;
        double angular_bias = 0.0;
        double position_bias_x = 0.0;
        double position_bias_y = 0.0;
        double theta_bias = 0.0;
    } noise_state_;
};
