#pragma once
#include <fstream>
#include <string>
#include <vector>
#include <cmath>
#include <random>
#include <memory>

#include <ros/ros.h>
#include <ros/package.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Vector3.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

#include <yaml-cpp/yaml.h>

#include "perception/Obstacle.h"
#include "perception/PredictedObstacles.h"


namespace Perception
{

class PerceptionSimulator
{
public:
    PerceptionSimulator();
    ~PerceptionSimulator() = default;

    void Initialize();
    void Run();

private:
    struct ObstacleConfig
    {
        int id;
        int type;
        double init_x;
        double init_y;
        double init_yaw;      // 弧度
        double length;
        double width;
        double height;
        double speed;         // m/s
        bool use_noise;       // 是否添加噪声
        double noise_stddev;  // 噪声标准差
        std::string name;     // 障碍物名称
    };

    struct ObstacleState
    {
        ObstacleConfig config;
        double current_x;
        double current_y;
        double current_yaw;
        double current_speed;
        std::default_random_engine generator;
        std::normal_distribution<double> distribution;

        ObstacleState(const ObstacleConfig & cfg);
        void Update(double dt);
    };

    struct TrajectoryVisualization
    {
        std::vector<geometry_msgs::Point> points;
        std::vector<double> times;
    };

private:
    void LoadParameters();
    void LoadObstacles(const std::string & config_file);
    void InitializePublishers();
    void InitializeTimer();

    void TimerCallback(const ros::TimerEvent & event);
    void UpdateObstacles(double dt);
    void PublishObstacles();
    void PublishVisualization();

    visualization_msgs::Marker CreateObstacleMarker(const ObstacleState & obstacle, int id) const;
    visualization_msgs::Marker CreateVelocityMarker(const ObstacleState & obstacle, int id) const;
    visualization_msgs::Marker CreateTrajectoryMarker(const ObstacleState & obstacle, int id) const;
    visualization_msgs::Marker CreateTextMarker(const ObstacleState & obstacle, int id) const;

    geometry_msgs::Point CalculateTrajectoryPoint(const ObstacleState & obstacle, double t) const;

private:
    ros::NodeHandle nh_;
    ros::NodeHandle private_nh_;

    // 参数
    std::string config_file_;
    double simulation_rate_;
    double prediction_time_step_;
    int prediction_points_;
    double marker_lifetime_;
    bool enable_visualization_;

    // 发布者
    ros::Publisher obstacles_pub_;
    ros::Publisher visualization_pub_;

    // 定时器
    ros::Timer timer_;

    // 数据
    std::vector<ObstacleState> obstacles_state_;
    ros::Time last_update_time_;

    // 颜色配置
    struct ColorConfig
    {
        float r, g, b, a;
    };

    std::map<int, ColorConfig> color_map_;

    void InitializeColorMap();
};

} // namespace Perception