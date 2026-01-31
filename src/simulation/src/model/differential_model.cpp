#include "simulation/model/differential_model.h"


DifferentialModel::DifferentialModel()
{
    // 默认参数
    max_linear_vel_ = 1.0;
    max_angular_vel_ = 1.0;

    // 默认噪声参数
    linear_noise_stddev_ = 0.02;
    angular_noise_stddev_ = 0.05;
    position_noise_stddev_ = 0.005;
    velocity_noise_stddev_ = 0.01;

    // 初始化噪声偏置（模拟系统固定偏差）
    noise_state_.linear_bias = 0.0;
    noise_state_.angular_bias = 0.0;
    noise_state_.position_bias_x = 0.0;
    noise_state_.position_bias_y = 0.0;
    noise_state_.theta_bias = 0.0;

    // 初始化随机数生成器
    std::random_device rd;
    random_generator_ = std::default_random_engine(rd());

    // 初始化分布
    linear_noise_dist_ = std::normal_distribution<double>(0.0, linear_noise_stddev_);
    angular_noise_dist_ = std::normal_distribution<double>(0.0, angular_noise_stddev_);
    position_noise_dist_ = std::normal_distribution<double>(0.0, position_noise_stddev_);
    velocity_noise_dist_ = std::normal_distribution<double>(0.0, velocity_noise_stddev_);
}

void DifferentialModel::initialize(double max_linear_vel, double max_angular_vel)
{
    max_linear_vel_ = max_linear_vel;
    max_angular_vel_ = max_angular_vel;
}

void DifferentialModel::setInitialState(double x, double y, double theta)
{
    initial_state_.x = x;
    initial_state_.y = y;
    initial_state_.theta = theta;
    initial_state_.v = 0.0;
    initial_state_.w = 0.0;
    initial_state_.timestamp = std::chrono::steady_clock::now();

    current_state_ = initial_state_;
}

void DifferentialModel::setNoiseParameters(double linear_noise_stddev,
    double angular_noise_stddev,
    double position_noise_stddev,
    double velocity_noise_stddev)
{
    linear_noise_stddev_ = linear_noise_stddev;
    angular_noise_stddev_ = angular_noise_stddev;
    position_noise_stddev_ = position_noise_stddev;
    velocity_noise_stddev_ = velocity_noise_stddev;

    // 重新初始化分布
    linear_noise_dist_ = std::normal_distribution<double>(0.0, linear_noise_stddev_);
    angular_noise_dist_ = std::normal_distribution<double>(0.0, angular_noise_stddev_);
    position_noise_dist_ = std::normal_distribution<double>(0.0, position_noise_stddev_);
    velocity_noise_dist_ = std::normal_distribution<double>(0.0, velocity_noise_stddev_);
}

void DifferentialModel::clampVelocity(double & linear_vel, double & angular_vel)
{
    if (linear_vel > max_linear_vel_) linear_vel = max_linear_vel_;
    if (linear_vel < -max_linear_vel_) linear_vel = -max_linear_vel_;
    if (angular_vel > max_angular_vel_) angular_vel = max_angular_vel_;
    if (angular_vel < -max_angular_vel_) angular_vel = -max_angular_vel_;
}

void DifferentialModel::normalizeAngle(double & angle)
{
    while (angle > M_PI) angle -= 2.0 * M_PI;
    while (angle < -M_PI) angle += 2.0 * M_PI;
}

void DifferentialModel::addControlNoise(double & linear_vel, double & angular_vel)
{
    // 添加高斯白噪声到控制输入
    linear_vel += linear_noise_dist_(random_generator_);
    angular_vel += angular_noise_dist_(random_generator_);

    // 添加固定偏置（模拟系统偏差）
    // 固定为0，直接注释掉了
    // linear_vel += noise_state_.linear_bias;
    // angular_vel += noise_state_.angular_bias;

    // 限制噪声后的控制输入
    clampVelocity(linear_vel, angular_vel);
}

void DifferentialModel::addProcessNoise(VehicleState & state, double dt)
{
    // 过程噪声：模拟运动模型的不确定性

    // 1. 位置漂移（随机游走）
    state.x += position_noise_dist_(random_generator_) * dt;
    state.y += position_noise_dist_(random_generator_) * dt;

    // 2. 角度漂移
    state.theta += angular_noise_dist_(random_generator_) * dt * 0.1;

    // 3. 速度随机波动
    state.v += velocity_noise_dist_(random_generator_) * dt;
    state.w += angular_noise_dist_(random_generator_) * dt;

    // 4. 累积偏置（模拟传感器漂移）
    noise_state_.position_bias_x += linear_noise_dist_(random_generator_) * dt * 0.01;
    noise_state_.position_bias_y += linear_noise_dist_(random_generator_) * dt * 0.01;
    noise_state_.theta_bias += angular_noise_dist_(random_generator_) * dt * 0.01;

    state.x += noise_state_.position_bias_x * dt;
    state.y += noise_state_.position_bias_y * dt;
    state.theta += noise_state_.theta_bias * dt;

    normalizeAngle(state.theta);
}

void DifferentialModel::addMeasurementNoise(VehicleState & state)
{
    // 测量噪声：模拟传感器噪声

    // 1. 位置测量噪声（高斯噪声）
    state.x += position_noise_dist_(random_generator_);
    state.y += position_noise_dist_(random_generator_);

    // 2. 角度测量噪声
    state.theta += angular_noise_dist_(random_generator_) * 0.05;

    // 3. 速度测量噪声
    state.v += velocity_noise_dist_(random_generator_);
    state.w += angular_noise_dist_(random_generator_);

    normalizeAngle(state.theta);
}

VehicleState DifferentialModel::updateIdeal(double linear_vel, double angular_vel, double dt)
{
    VehicleState new_state = current_state_;

    // 限制控制输入
    clampVelocity(linear_vel, angular_vel);

    // 差速运动学模型（理想）
    double v = linear_vel;
    double w = angular_vel;

    if (std::abs(w) < 1e-6)
    {
        // 直线运动
        new_state.x += v * std::cos(new_state.theta) * dt;
        new_state.y += v * std::sin(new_state.theta) * dt;
    }
    else
    {
        // 圆弧运动
        double r = v / w;
        double delta_theta = w * dt;

        new_state.x += r * (std::sin(new_state.theta + delta_theta) -
            std::sin(new_state.theta));
        new_state.y += -r * (std::cos(new_state.theta + delta_theta) -
            std::cos(new_state.theta));
        new_state.theta += delta_theta;
    }

    normalizeAngle(new_state.theta);

    // 更新速度
    new_state.v = v;
    new_state.w = w;
    new_state.timestamp = std::chrono::steady_clock::now();

    return new_state;
}

VehicleState DifferentialModel::updateNoisy(double linear_vel, double angular_vel, double dt)
{
    // 复制当前状态作为基础
    VehicleState new_state = current_state_;

    // 1. 添加控制噪声
    double noisy_linear_vel = linear_vel;
    double noisy_angular_vel = angular_vel;
    addControlNoise(noisy_linear_vel, noisy_angular_vel);

    // 2. 运动模型更新（使用带噪声的控制输入）
    double v = noisy_linear_vel;
    double w = noisy_angular_vel;

    if (std::abs(w) < 1e-6)
    {
        new_state.x += v * std::cos(new_state.theta) * dt;
        new_state.y += v * std::sin(new_state.theta) * dt;
    }
    else
    {
        double r = v / w;
        double delta_theta = w * dt;

        new_state.x += r * (std::sin(new_state.theta + delta_theta) -
            std::sin(new_state.theta));
        new_state.y += -r * (std::cos(new_state.theta + delta_theta) -
            std::cos(new_state.theta));
        new_state.theta += delta_theta;
    }

    normalizeAngle(new_state.theta);

    // 3. 添加过程噪声
    addProcessNoise(new_state, dt);

    // 4. 添加测量噪声
    addMeasurementNoise(new_state);

    // 更新速度（使用带噪声的速度）
    new_state.v = v;
    new_state.w = w;
    new_state.timestamp = std::chrono::steady_clock::now();

    return new_state;
}

VehicleState DifferentialModel::update(double linear_vel, double angular_vel,
    double dt, bool add_noise)
{
    if (add_noise)
    {
        current_state_ = updateNoisy(linear_vel, angular_vel, dt);
    }
    else
    {
        current_state_ = updateIdeal(linear_vel, angular_vel, dt);
    }

    return current_state_;
}

void DifferentialModel::reset()
{
    current_state_ = initial_state_;
    current_state_.timestamp = std::chrono::steady_clock::now();

    // 重置噪声状态
    noise_state_ = {};
}

VehicleState DifferentialModel::getCurrentState() const
{
    return current_state_;
}