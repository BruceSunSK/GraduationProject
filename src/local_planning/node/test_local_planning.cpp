#include <fstream>
#include <iomanip>
#include <ctime>
#include <sstream>
#include <sys/stat.h>
#include <errno.h>
#include <chrono>

#include <ros/ros.h>
#include <ros/package.h>
#include <nav_msgs/Odometry.h>
#include <tf2/utils.h>

#include "local_planning/local_planning.h"

// 派生类，增加数据记录功能
class TestLocalPlanning : public LocalPlanning
{
public:
    TestLocalPlanning(ros::NodeHandle & nh, const ros::Rate & loop_rate)
        : LocalPlanning(nh, loop_rate)
        , vehicle_data_file_(nullptr)
        , planning_time_file_(nullptr)
        , last_odom_timestamp_(0.0)
        , last_odom_v_(0.0)
        , last_record_time_(0.0)
        , planning_count_(0)
    {
        // 从参数服务器读取记录间隔（秒），默认为 1.0
        nh.param("record_interval", record_interval_, 1.0);

        // 创建保存数据的目录（基于时间戳）
        std::string package_path = ros::package::getPath("local_planning");
        if (package_path.empty())
        {
            ROS_ERROR("[TestLocalPlanning]: Failed to get package path");
            return;
        }
        std::string timestamp = getCurrentTimestamp();
        data_dir_ = package_path + "/result/test/local_planning/" + timestamp + "/";
        if (!createDirectoryRecursively(data_dir_))
        {
            ROS_ERROR("[TestLocalPlanning]: Failed to create directory %s", data_dir_.c_str());
            return;
        }

        // 打开车辆数据文件
        std::string vehicle_filename = data_dir_ + "vehicle_data.csv";
        vehicle_data_file_.open(vehicle_filename);
        if (!vehicle_data_file_.is_open())
        {
            ROS_ERROR("[TestLocalPlanning]: Failed to open file %s", vehicle_filename.c_str());
            return;
        }
        vehicle_data_file_ << "timestamp,x,y,theta,v,ax,ay,kappa\n";
        vehicle_data_file_ << std::fixed << std::setprecision(6);
        ROS_INFO("[TestLocalPlanning]: Vehicle data will be recorded to %s", vehicle_filename.c_str());

        // 打开规划耗时文件
        std::string planning_time_filename = data_dir_ + "planning_times.csv";
        planning_time_file_.open(planning_time_filename);
        if (!planning_time_file_.is_open())
        {
            ROS_ERROR("[TestLocalPlanning]: Failed to open file %s", planning_time_filename.c_str());
            return;
        }
        planning_time_file_ << "index,cost_time_ms\n";
        planning_time_file_ << std::fixed << std::setprecision(3);
        ROS_INFO("[TestLocalPlanning]: Planning times will be recorded to %s", planning_time_filename.c_str());

        if (record_interval_ > 0)
            ROS_INFO("[TestLocalPlanning]: Vehicle data recording interval: %.3f s", record_interval_);
        else
            ROS_INFO("[TestLocalPlanning]: Vehicle data recorded every frame");
    }

    ~TestLocalPlanning()
    {
        if (vehicle_data_file_.is_open())
            vehicle_data_file_.close();
        if (planning_time_file_.is_open())
            planning_time_file_.close();
    }

protected:
    // 重写车辆状态回调，记录车辆数据
    void VehicleStateCallback(const nav_msgs::Odometry::ConstPtr & msg) override
    {
        // 先调用基类处理（更新 planner 中的车辆状态）
        LocalPlanning::VehicleStateCallback(msg);

        double timestamp = msg->header.stamp.toSec();

        // 判断是否需要记录车辆数据
        bool should_record = false;
        if (record_interval_ <= 0.0)
        {
            should_record = true;  // 记录所有帧
        }
        else
        {
            if (last_record_time_ == 0.0 || (timestamp - last_record_time_ >= record_interval_))
            {
                should_record = true;
            }
        }

        if (should_record)
        {
            // 提取数据
            double x = msg->pose.pose.position.x;
            double y = msg->pose.pose.position.y;
            double theta = tf2::getYaw(msg->pose.pose.orientation);
            double v = msg->twist.twist.linear.x;
            double w = msg->twist.twist.angular.z;
            double kappa = (std::abs(v) > 1e-3) ? (w / v) : 0.0;

            // 计算加速度
            double ax = 0.0, ay = 0.0;
            if (last_odom_timestamp_ > 0)
            {
                double dt = timestamp - last_odom_timestamp_;
                if (dt > 0)
                {
                    ax = (v - last_odom_v_) / dt;    // 纵向加速度
                    ay = v * w;                      // 横向加速度（向心加速度近似）
                }
            }

            // 写入文件
            if (vehicle_data_file_.is_open())
            {
                vehicle_data_file_ << timestamp << ","
                                   << x << "," << y << "," << theta << ","
                                   << v << "," << ax << "," << ay << ","
                                   << kappa << "\n";
            }

            // 更新上一次记录时间
            last_record_time_ = timestamp;
        }

        // 更新上一次里程计数据（用于加速度计算）
        last_odom_timestamp_ = timestamp;
        last_odom_v_ = msg->twist.twist.linear.x;
    }

    // 重写规划完成回调，记录规划耗时
    void OnPlanningCompleted(const LocalPlanner::LocalPlannerResult & result) override
    {
        if (planning_time_file_.is_open())
        {
            // result.planning_cost_time 单位为秒，转换为毫秒
            planning_time_file_ << planning_count_++ << "," << result.planning_cost_time * 1000.0 << "\n";
        }
    }

private:
    // 数据文件流
    std::ofstream vehicle_data_file_;
    std::ofstream planning_time_file_;
    std::string data_dir_;                 // 数据保存目录

    // 车辆数据记录相关
    double last_odom_timestamp_;            // 上一次里程计时间戳
    double last_odom_v_;                    // 上一次速度（用于加速度计算）
    double last_record_time_;                // 上次记录车辆数据的时间戳
    double record_interval_;                 // 车辆数据记录间隔（秒）

    // 规划耗时记录相关
    int planning_count_;                     // 规划次数计数

    // 辅助函数：获取当前时间戳字符串（格式：YYYYMMDD_HHMMSS）
    static std::string getCurrentTimestamp()
    {
        auto now = std::chrono::system_clock::now();
        auto in_time_t = std::chrono::system_clock::to_time_t(now);
        std::stringstream ss;
        ss << std::put_time(std::localtime(&in_time_t), "%Y%m%d_%H%M%S");
        return ss.str();
    }

    // 辅助函数：递归创建目录
    static bool createDirectoryRecursively(const std::string & path)
    {
        size_t pos = 0;
        std::string dir;
        int ret;
        while (pos < path.length())
        {
            pos = path.find('/', pos + 1);
            dir = path.substr(0, pos);
            if (dir.empty()) continue;
            ret = mkdir(dir.c_str(), 0755);
            if (ret != 0 && errno != EEXIST)
            {
                ROS_ERROR("[TestLocalPlanning]: Failed to create directory %s: %s",
                          dir.c_str(), strerror(errno));
                return false;
            }
        }
        return true;
    }
};

// main 函数
int main(int argc, char ** argv)
{
    ros::init(argc, argv, "test_local_planning");
    ros::NodeHandle nh("~");
    ros::Rate loop_rate(10);   // 控制主循环频率（与 LocalPlanning 中的循环速率一致）

    TestLocalPlanning planning(nh, loop_rate);
    planning.Run();
    return 0;
}