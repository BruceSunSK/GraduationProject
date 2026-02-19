#include <ros/ros.h>
#include <ros/package.h>
#include <fstream>
#include <iomanip>
#include <sys/stat.h>
#include <ctime>
#include <sstream>
#include <string>
#include <vector>
#include <algorithm>

#include "global_planning/smoothers/piecewise_jerk_speed_smoother.h"

using namespace Smoother;

enum class ObstacleType
{
    STATIC,
    DYNAMIC
};

struct Obstacle
{
    ObstacleType type;
    double s_start;          // 起始纵向位置 (m)
    double s_end;            // 结束纵向位置 (m) —— 静态障碍物使用
    double length;           // 障碍物长度 (m) —— 用于动态障碍物
    double velocity;         // 障碍物速度 (m/s) —— 用于动态障碍物
    double t_start;          // 障碍物出现时间 (s)
    double t_end;            // 障碍物消失时间 (s)
    double safety_margin;    // 安全距离 (m)
    bool overtake;           // 是否超车（仅动态障碍物有效）

    // 静态障碍物构造函数
    Obstacle(ObstacleType t, double s, double len, double margin = 2.0)
        : type(t), s_start(s), length(len), velocity(0), t_start(0), t_end(1e9), safety_margin(margin), overtake(false)
    {
        s_end = s + len;
    }

    // 动态障碍物构造函数
    Obstacle(ObstacleType t, double s, double len, double v, double t0, double t1, bool overtake_flag, double margin = 2.0)
        : type(t), s_start(s), length(len), velocity(v), t_start(t0), t_end(t1), safety_margin(margin), overtake(overtake_flag)
    {}
};

void ApplyObstacles(const std::vector<Obstacle> & obstacles,
    double dt, int N,
    std::vector<double> & s_upper,
    std::vector<double> & s_lower,
    std::vector<double> & v_upper,
    std::vector<double> & v_lower,
    std::vector<double> & kappa_ref)
{
    // 初始化边界（s_upper 已在外部设为 s_end）
    // s_lower 已在外部设为 0.0

    for (int i = 0; i < N; ++i)
    {
        double t = i * dt;
        double s_lower_i = s_lower[i];
        double s_upper_i = s_upper[i];

        for (const auto & obs : obstacles)
        {
            if (obs.type == ObstacleType::STATIC)
            {
                if (obs.s_start > 0)
                {
                    s_upper_i = std::min(s_upper_i, obs.s_start - obs.safety_margin);
                }
            }
            else if (obs.type == ObstacleType::DYNAMIC)
            {
                if (t >= obs.t_start && t <= obs.t_end)
                {
                    double s_obs_center = obs.s_start + obs.velocity * (t - obs.t_start);
                    double s_obs_front = s_obs_center + obs.length / 2.0;
                    double s_obs_rear = s_obs_center - obs.length / 2.0;

                    if (obs.overtake)
                    {
                        // 超车：车辆应在障碍物前方，即 s >= s_obs_front + margin
                        s_lower_i = std::max(s_lower_i, s_obs_front + obs.safety_margin);
                    }
                    else
                    {
                        // 跟车：车辆应在障碍物后方，即 s <= s_obs_rear - margin
                        if (s_obs_rear - obs.safety_margin > 0)
                        {
                            s_upper_i = std::min(s_upper_i, s_obs_rear - obs.safety_margin);
                        }
                    }
                }
            }
        }

        // 检测并处理冲突
        if (s_lower_i > s_upper_i)
        {
            ROS_WARN("Conflict at time t=%.2f: s_lower=%.2f > s_upper=%.2f", t, s_lower_i, s_upper_i);
            // 保守调整：令可行域退化为一个点（取中值）
            double mid = (s_lower_i + s_upper_i) / 2.0;
            s_lower_i = mid;
            s_upper_i = mid;
        }

        s_lower[i] = s_lower_i;
        s_upper[i] = s_upper_i;
    }
}

std::string GetCurrentTimestamp()
{
    std::time_t t = std::time(nullptr);
    std::tm tm = *std::localtime(&t);
    std::ostringstream oss;
    oss << std::put_time(&tm, "%Y%m%d_%H%M%S");
    return oss.str();
}

bool EnsureDirectory(const std::string & path)
{
    struct stat info;
    if (stat(path.c_str(), &info) != 0)
    {
        if (mkdir(path.c_str(), 0755) != 0)
        {
            ROS_ERROR("Failed to create directory: %s", path.c_str());
            return false;
        }
    }
    else if (!(info.st_mode & S_IFDIR))
    {
        ROS_ERROR("%s exists but is not a directory", path.c_str());
        return false;
    }
    return true;
}

void WriteObstacleComment(std::ofstream & file, const std::vector<Obstacle> & obstacles)
{
    file << "# obstacles: ";
    for (const auto & obs : obstacles)
    {
        if (obs.type == ObstacleType::STATIC)
        {
            file << "static[" << obs.s_start << "-" << obs.s_end << ",margin=" << obs.safety_margin << "] ";
        }
        else
        {
            file << "dynamic[s0=" << obs.s_start << ",v=" << obs.velocity
                << ",len=" << obs.length << ",t=" << obs.t_start << "-" << obs.t_end
                << ",overtake=" << (obs.overtake ? "true" : "false")
                << ",margin=" << obs.safety_margin << "] ";
        }
    }
    file << "\n";
}

int main(int argc, char ** argv)
{
    ros::init(argc, argv, "test_speed_planning");
    ros::NodeHandle nh("~");

    // 参数读取
    double dt, w_acc, w_jerk, w_speed_dev, w_lat, s0, v0, a0, s_end;
    int N;
    std::string scenario;

    nh.param<double>("dt", dt, 0.1);
    nh.param<double>("w_acceleration", w_acc, 1.0);
    nh.param<double>("w_jerk", w_jerk, 5.0);
    nh.param<double>("w_speed_deviation", w_speed_dev, 10.0);
    nh.param<double>("w_lateral_accel", w_lat, 0.1);
    nh.param<double>("s0", s0, 0.0);
    nh.param<double>("v0", v0, 5.0);
    nh.param<double>("a0", a0, 0.0);
    nh.param<double>("s_end", s_end, 60.0);
    nh.param<int>("N", N, 80);
    nh.param<std::string>("scenario", scenario, "default");

    // 定义障碍物
    std::vector<Obstacle> obstacles;
    if (scenario == "static_obstacle")
    {
        obstacles.emplace_back(ObstacleType::STATIC, 20.0, 5.0, 2.0);
    }
    else if (scenario == "dynamic_obstacle")
    {
        obstacles.emplace_back(ObstacleType::DYNAMIC, 25.0, 4.0, 2.0, 1.0, 8.0, false, 2.0);
    }
    else if (scenario == "multiple")
    {
        obstacles.emplace_back(ObstacleType::STATIC, 20.0, 5.0, 2.0);
        obstacles.emplace_back(ObstacleType::DYNAMIC, 35.0, 4.0, 3.0, 2.0, 7.0, false, 2.0);
    }
    else if (scenario == "two_dynamic")
    {
        // 第一个动态障碍物：低速，超车
        obstacles.emplace_back(ObstacleType::DYNAMIC, 25.0, 4.0, 1.5, 4.0, 12.0, true, 2.0);
        // 第二个动态障碍物：较高速，跟车
        obstacles.emplace_back(ObstacleType::DYNAMIC, 35.0, 4.0, 3.0, 1.0, 15.0, false, 2.0);
    }

    // 初始化边界，s_upper 默认 s_end
    std::vector<double> s_lower(N, 0.0), s_upper(N, s_end);
    std::vector<double> v_lower(N, 0.0), v_upper(N, 20.0);
    std::vector<double> a_lower(N, -3.0), a_upper(N, 2.0);
    std::vector<double> v_ref(N, 8.0), kappa_ref(N, 0.0);

    // 应用障碍物约束
    ApplyObstacles(obstacles, dt, N, s_upper, s_lower, v_upper, v_lower, kappa_ref);

    // 创建优化器
    PiecewiseJerkSpeedSmoother::Weights weights;
    weights.w_acceleration = w_acc;
    weights.w_jerk = w_jerk;
    weights.w_speed_deviation = w_speed_dev;
    weights.w_lateral_acceleration = w_lat;
    PiecewiseJerkSpeedSmoother smoother(dt, weights);

    std::vector<Path::TrajectoryPoint> result;
    if (!smoother.Solve(s0, v0, a0, s_lower, s_upper, v_lower, v_upper,
        a_lower, a_upper, v_ref, kappa_ref, s_end, result))
    {
        ROS_ERROR("Speed planning failed!");
        return 1;
    }

    // 创建实验文件夹：local_planning/result/test/speed_planning/时间戳_场景名/
    std::string pkg_path = ros::package::getPath("local_planning");
    if (pkg_path.empty())
    {
        ROS_ERROR("Failed to get package path");
        return 1;
    }
    std::string base_dir = pkg_path + "/result/test/speed_planning/";
    if (!EnsureDirectory(base_dir)) return 1;

    std::string folder_name = GetCurrentTimestamp() + "_" + scenario;
    std::string exp_dir = base_dir + folder_name + "/";
    if (!EnsureDirectory(exp_dir)) return 1;

    std::string csv_path = exp_dir + "data.csv";
    std::ofstream file(csv_path);
    if (!file.is_open())
    {
        ROS_ERROR("Cannot open file %s", csv_path.c_str());
        return 1;
    }

    WriteObstacleComment(file, obstacles);
    file << "t,s,v,a,s_lower,s_upper,v_lower,v_upper,a_lower,a_upper\n";

    for (size_t i = 0; i < result.size(); ++i)
    {
        const auto & p = result[i];
        file << std::fixed << std::setprecision(6)
            << p.t << "," << p.s << "," << p.v << "," << p.a << ","
            << s_lower[i] << "," << s_upper[i] << ","
            << v_lower[i] << "," << v_upper[i] << ","
            << a_lower[i] << "," << a_upper[i] << "\n";
    }
    file.close();
    ROS_INFO("Results saved to %s", csv_path.c_str());

    return 0;
}