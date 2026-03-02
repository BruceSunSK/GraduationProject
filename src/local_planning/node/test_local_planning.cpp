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
        , data_file_(nullptr)
        , last_timestamp_(0.0)
        , last_v_(0.0)
        , last_record_time_(0.0)
    {
        // 从参数服务器读取记录间隔（秒），默认为0，表示记录所有帧
        nh.param("record_interval", record_interval_, 1.0);

        // 创建数据文件
        std::string package_path = ros::package::getPath("local_planning");
        if (package_path.empty())
        {
            ROS_ERROR("[TestLocalPlanning]: Failed to get package path");
            return;
        }
        std::string timestamp = getCurrentTimestamp();
        std::string dir = package_path + "/result/test/local_planning/" + timestamp + "/";
        if (!createDirectoryRecursively(dir))
        {
            ROS_ERROR("[TestLocalPlanning]: Failed to create directory %s", dir.c_str());
            return;
        }
        std::string filename = dir + "vehicle_data.csv";
        data_file_.open(filename);
        if (!data_file_.is_open())
        {
            ROS_ERROR("[TestLocalPlanning]: Failed to open file %s", filename.c_str());
            return;
        }
        // 写入表头
        data_file_ << "timestamp,x,y,theta,v,ax,ay,kappa\n";
        data_file_ << std::fixed << std::setprecision(6);

        if (record_interval_ > 0)
            ROS_INFO("[TestLocalPlanning]: Data recording with interval %.3f s to %s", record_interval_, filename.c_str());
        else
            ROS_INFO("[TestLocalPlanning]: Data recording every frame to %s", filename.c_str());
    }

    ~TestLocalPlanning()
    {
        if (data_file_.is_open())
            data_file_.close();
    }

protected:
    // 重写车辆状态回调，记录数据
    void VehicleStateCallback(const nav_msgs::Odometry::ConstPtr & msg) override
    {
        // 先调用基类处理（更新planner中的车辆状态）
        LocalPlanning::VehicleStateCallback(msg);

        double timestamp = msg->header.stamp.toSec();

        // 判断是否需要记录
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
            if (last_timestamp_ > 0)
            {
                double dt = timestamp - last_timestamp_;
                if (dt > 0)
                {
                    ax = (v - last_v_) / dt;          // 纵向加速度
                    ay = v * w;                        // 横向加速度（向心加速度近似）
                }
            }

            // 写入文件
            if (data_file_.is_open())
            {
                data_file_ << timestamp << ","
                    << x << "," << y << "," << theta << ","
                    << v << "," << ax << "," << ay << ","
                    << kappa << "\n";
            }

            // 更新上一次记录时间
            last_record_time_ = timestamp;
        }

        // 无论是否记录，都需要更新这些值用于下一次加速度计算
        last_timestamp_ = timestamp;
        last_v_ = msg->twist.twist.linear.x;
    }

private:
    std::ofstream data_file_;
    double last_timestamp_;
    double last_v_;
    double last_record_time_;      // 上次记录的时间戳
    double record_interval_;       // 记录间隔（秒）

    // 辅助函数：获取当前时间戳字符串
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
                ROS_ERROR("[TestLocalPlanning]: Failed to create directory %s: %s", dir.c_str(), strerror(errno));
                return false;
            }
        }
        return true;
    }
};

// main函数
int main(int argc, char ** argv)
{
    ros::init(argc, argv, "test_local_planning");
    ros::NodeHandle nh("~");
    ros::Rate loop_rate(10);

    TestLocalPlanning planning(nh, loop_rate);
    planning.Run();
    return 0;
}