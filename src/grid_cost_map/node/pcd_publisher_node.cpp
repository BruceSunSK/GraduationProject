#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/uniform_sampling.h>
#include <pcl/filters/random_sample.h>
#include <pcl/filters/approximate_voxel_grid.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/filters/radius_outlier_removal.h>

class PCDPublisher
{
private:
    ros::NodeHandle nh_;
    ros::Publisher cloud_pub_;
    std::string frame_id_;
    std::string pcd_file_path_;
    std::string topic_name_;

    // XYZ三轴范围过滤参数
    bool use_range_filter_;
    bool filter_x_axis_;
    bool filter_y_axis_;
    bool filter_z_axis_;
    float min_x_;
    float max_x_;
    float min_y_;
    float max_y_;
    float min_z_;
    float max_z_;

    // 离群点过滤参数
    bool use_outlier_removal_;
    std::string outlier_removal_method_;
    int statistical_mean_k_;
    float statistical_stddev_mult_;
    float radius_search_radius_;
    int radius_min_neighbors_;

    // 降采样参数
    bool use_downsampling_;
    std::string downsampling_method_;
    float voxel_leaf_size_;
    float uniform_search_radius_;
    int random_sample_num_;
    bool use_approximate_voxel_;
    bool display_filter_stats_;

public:
    PCDPublisher()
    {
        // 从参数服务器获取参数
        ros::NodeHandle private_nh("~");

        // 获取PCD文件路径（必需参数）
        if (!private_nh.getParam("pcd_file", pcd_file_path_))
        {
            ROS_ERROR("Need parameter 'pcd_file' !!");
            ros::shutdown();
            return;
        }

        private_nh.param<std::string>("topic_name", topic_name_, "pointcloud");
        private_nh.param<std::string>("frame_id", frame_id_, "map");

        // XYZ三轴范围过滤参数
        private_nh.param<bool>("use_range_filter", use_range_filter_, true);
        private_nh.param<bool>("filter_x_axis", filter_x_axis_, false);
        private_nh.param<bool>("filter_y_axis", filter_y_axis_, false);
        private_nh.param<bool>("filter_z_axis", filter_z_axis_, true);
        private_nh.param<float>("min_x", min_x_, -100.0f);
        private_nh.param<float>("max_x", max_x_, 100.0f);
        private_nh.param<float>("min_y", min_y_, -100.0f);
        private_nh.param<float>("max_y", max_y_, 100.0f);
        private_nh.param<float>("min_z", min_z_, -10.0f);
        private_nh.param<float>("max_z", max_z_, 3.0f);

        // 离群点过滤参数
        private_nh.param<bool>("use_outlier_removal", use_outlier_removal_, false);
        private_nh.param<std::string>("outlier_removal_method", outlier_removal_method_, "statistical");
        private_nh.param<int>("statistical_mean_k", statistical_mean_k_, 50);
        private_nh.param<float>("statistical_stddev_mult", statistical_stddev_mult_, 1.0f);
        private_nh.param<float>("radius_search_radius", radius_search_radius_, 0.1f);
        private_nh.param<int>("radius_min_neighbors", radius_min_neighbors_, 5);

        // 降采样参数
        private_nh.param<bool>("use_downsampling", use_downsampling_, true);
        private_nh.param<std::string>("downsampling_method", downsampling_method_, "voxel");
        private_nh.param<float>("voxel_leaf_size", voxel_leaf_size_, 0.05f);
        private_nh.param<float>("uniform_search_radius", uniform_search_radius_, 0.1f);
        private_nh.param<int>("random_sample_num", random_sample_num_, 10000);
        private_nh.param<bool>("use_approximate_voxel", use_approximate_voxel_, false);
        private_nh.param<bool>("display_filter_stats", display_filter_stats_, true);

        // 初始化发布者
        cloud_pub_ = nh_.advertise<sensor_msgs::PointCloud2>(topic_name_, 1, true);

        // 显示节点初始化信息
        ROS_INFO("========================================");
        ROS_INFO("PCD Publisher Node Initialized");
        ROS_INFO("========================================");
        ROS_INFO("PCD file: %s", pcd_file_path_.c_str());
        ROS_INFO("Frame ID: %s", frame_id_.c_str());
        ROS_INFO("Topic name: %s", topic_name_.c_str());
        ROS_INFO("----------------------------------------");

        // 显示处理参数
        ROS_INFO("Processing Parameters:");
        ROS_INFO("  Range filter: %s", use_range_filter_ ? "ON" : "OFF");
        if (use_range_filter_)
        {
            if (filter_x_axis_) ROS_INFO("    X axis: [%.2f, %.2f]", min_x_, max_x_);
            if (filter_y_axis_) ROS_INFO("    Y axis: [%.2f, %.2f]", min_y_, max_y_);
            if (filter_z_axis_) ROS_INFO("    Z axis: [%.2f, %.2f]", min_z_, max_z_);
        }

        ROS_INFO("  Outlier removal: %s", use_outlier_removal_ ? "ON" : "OFF");
        if (use_outlier_removal_)
            ROS_INFO("    Method: %s", outlier_removal_method_.c_str());

        ROS_INFO("  Downsampling: %s", use_downsampling_ ? "ON" : "OFF");
        if (use_downsampling_)
        {
            ROS_INFO("    Method: %s", downsampling_method_.c_str());
            if (downsampling_method_ == "voxel")
                ROS_INFO("    Leaf size: %.3f m", voxel_leaf_size_);
            else if (downsampling_method_ == "uniform")
                ROS_INFO("    Radius: %.3f m", uniform_search_radius_);
            else if (downsampling_method_ == "random")
                ROS_INFO("    Target points: %d", random_sample_num_);
        }
        ROS_INFO("========================================");
    }

    pcl::PointCloud<pcl::PointXYZ>::Ptr loadPCD()
    {
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);

        // 加载PCD文件
        if (pcl::io::loadPCDFile<pcl::PointXYZ>(pcd_file_path_, *cloud) == -1)
        {
            ROS_ERROR("Cannot load PCD file: %s", pcd_file_path_.c_str());
            return pcl::PointCloud<pcl::PointXYZ>::Ptr();
        }

        ROS_INFO("Loaded PCD file");
        ROS_INFO("  Original points: %ld", cloud->points.size());

        return cloud;
    }

    /**
     * @brief 应用XYZ三轴范围过滤到点云
     * @param cloud 输入点云
     * @return 范围过滤后的点云
     */
    pcl::PointCloud<pcl::PointXYZ>::Ptr applyRangeFilter(const pcl::PointCloud<pcl::PointXYZ>::Ptr & cloud)
    {
        if (!cloud || cloud->empty() || !use_range_filter_)
            return cloud;

        pcl::PointCloud<pcl::PointXYZ>::Ptr filtered_cloud(new pcl::PointCloud<pcl::PointXYZ>(*cloud));
        size_t original_size = cloud->points.size();

        // 使用直通滤波器进行三轴范围过滤
        pcl::PassThrough<pcl::PointXYZ> pass_filter;

        // X轴过滤
        if (filter_x_axis_)
        {
            pass_filter.setInputCloud(filtered_cloud);
            pass_filter.setFilterFieldName("x");
            pass_filter.setFilterLimits(min_x_, max_x_);
            pass_filter.filter(*filtered_cloud);
        }

        // Y轴过滤
        if (filter_y_axis_)
        {
            pass_filter.setInputCloud(filtered_cloud);
            pass_filter.setFilterFieldName("y");
            pass_filter.setFilterLimits(min_y_, max_y_);
            pass_filter.filter(*filtered_cloud);
        }

        // Z轴过滤
        if (filter_z_axis_)
        {
            pass_filter.setInputCloud(filtered_cloud);
            pass_filter.setFilterFieldName("z");
            pass_filter.setFilterLimits(min_z_, max_z_);
            pass_filter.filter(*filtered_cloud);
        }

        // 显示过滤统计信息
        if (display_filter_stats_ && !filtered_cloud->empty())
        {
            size_t filtered_size = filtered_cloud->points.size();
            size_t removed_count = original_size - filtered_size;
            float removal_ratio = 100.0f * static_cast<float>(removed_count) / original_size;

            ROS_INFO("Range filtering complete");
            ROS_INFO("  Removed points: %ld (%.1f%%)", removed_count, removal_ratio);
            ROS_INFO("  Remaining points: %ld", filtered_size);

            if (filtered_size == 0)
            {
                ROS_WARN("Range filtering resulted in empty point cloud!");
            }
        }

        return filtered_cloud;
    }

    /**
     * @brief 应用离群点移除
     * @param cloud 输入点云
     * @return 移除离群点后的点云
     */
    pcl::PointCloud<pcl::PointXYZ>::Ptr applyOutlierRemoval(const pcl::PointCloud<pcl::PointXYZ>::Ptr & cloud)
    {
        if (!cloud || cloud->empty() || !use_outlier_removal_)
            return cloud;

        pcl::PointCloud<pcl::PointXYZ>::Ptr filtered_cloud(new pcl::PointCloud<pcl::PointXYZ>);
        size_t original_size = cloud->points.size();

        if (outlier_removal_method_ == "statistical")
        {
            // 使用统计离群点移除
            pcl::StatisticalOutlierRemoval<pcl::PointXYZ> sor_filter;
            sor_filter.setInputCloud(cloud);
            sor_filter.setMeanK(statistical_mean_k_);
            sor_filter.setStddevMulThresh(statistical_stddev_mult_);
            sor_filter.filter(*filtered_cloud);
        }
        else if (outlier_removal_method_ == "radius")
        {
            // 使用半径离群点移除
            pcl::RadiusOutlierRemoval<pcl::PointXYZ> ror_filter;
            ror_filter.setInputCloud(cloud);
            ror_filter.setRadiusSearch(radius_search_radius_);
            ror_filter.setMinNeighborsInRadius(radius_min_neighbors_);
            ror_filter.filter(*filtered_cloud);
        }
        else
        {
            ROS_WARN("Unknown outlier removal method: %s. No outlier removal applied.", outlier_removal_method_.c_str());
            return cloud;
        }

        // 显示过滤统计信息
        if (display_filter_stats_ && !filtered_cloud->empty())
        {
            size_t filtered_size = filtered_cloud->points.size();
            size_t removed_count = original_size - filtered_size;
            float removal_ratio = 100.0f * static_cast<float>(removed_count) / original_size;

            ROS_INFO("Outlier removal complete");
            ROS_INFO("  Removed points: %ld (%.1f%%)", removed_count, removal_ratio);
            ROS_INFO("  Remaining points: %ld", filtered_size);

            if (filtered_size == 0)
            {
                ROS_WARN("Outlier removal resulted in empty point cloud!");
            }
        }

        return filtered_cloud;
    }

    /**
     * @brief 应用降采样到点云
     * @param cloud 输入点云
     * @return 降采样后的点云
     */
    pcl::PointCloud<pcl::PointXYZ>::Ptr downsampleCloud(const pcl::PointCloud<pcl::PointXYZ>::Ptr & cloud)
    {
        if (!cloud || cloud->empty() || !use_downsampling_)
            return cloud;

        pcl::PointCloud<pcl::PointXYZ>::Ptr downsampled_cloud(new pcl::PointCloud<pcl::PointXYZ>);
        size_t original_size = cloud->points.size();

        // 根据选择的方法进行降采样
        if (downsampling_method_ == "voxel")
        {
            if (use_approximate_voxel_)
            {
                // 使用近似体素栅格降采样（速度更快）
                pcl::ApproximateVoxelGrid<pcl::PointXYZ> approximate_voxel_filter;
                approximate_voxel_filter.setLeafSize(voxel_leaf_size_, voxel_leaf_size_, voxel_leaf_size_);
                approximate_voxel_filter.setInputCloud(cloud);
                approximate_voxel_filter.filter(*downsampled_cloud);
            }
            else
            {
                // 使用精确体素栅格降采样
                pcl::VoxelGrid<pcl::PointXYZ> voxel_filter;
                voxel_filter.setLeafSize(voxel_leaf_size_, voxel_leaf_size_, voxel_leaf_size_);
                voxel_filter.setInputCloud(cloud);
                voxel_filter.filter(*downsampled_cloud);
            }
        }
        else if (downsampling_method_ == "uniform")
        {
            // 使用均匀采样
            pcl::UniformSampling<pcl::PointXYZ> uniform_filter;
            uniform_filter.setInputCloud(cloud);
            uniform_filter.setRadiusSearch(uniform_search_radius_);
            uniform_filter.filter(*downsampled_cloud);
        }
        else if (downsampling_method_ == "random")
        {
            // 使用随机采样
            pcl::RandomSample<pcl::PointXYZ> random_filter;
            random_filter.setInputCloud(cloud);

            // 确保目标点数不超过原始点数
            int target_num = std::min(random_sample_num_, static_cast<int>(original_size));
            random_filter.setSample(target_num);
            random_filter.filter(*downsampled_cloud);
        }
        else
        {
            ROS_WARN("Unknown downsampling method: %s. No downsampling applied.", downsampling_method_.c_str());
            return cloud;
        }

        // 显示降采样统计信息
        if (display_filter_stats_ && !downsampled_cloud->empty())
        {
            size_t downsampled_size = downsampled_cloud->points.size();
            float reduction_ratio = 100.0f * (1.0f - static_cast<float>(downsampled_size) / original_size);
            ROS_INFO("Downsampling complete");
            ROS_INFO("  Removed points: %ld (%.1f%%)", original_size - downsampled_size, reduction_ratio);
            ROS_INFO("  Remaining points: %ld", downsampled_size);

            if (downsampled_size == 0)
            {
                ROS_WARN("Downsampling resulted in empty point cloud!");
                return cloud;
            }
        }

        return downsampled_cloud;
    }

    /**
     * @brief 处理点云的完整流程
     * @param cloud 原始点云
     * @return 处理后的点云
     */
    pcl::PointCloud<pcl::PointXYZ>::Ptr processPointCloud(const pcl::PointCloud<pcl::PointXYZ>::Ptr & cloud)
    {
        if (!cloud || cloud->empty())
            return cloud;

        pcl::PointCloud<pcl::PointXYZ>::Ptr processed_cloud = cloud;
        size_t original_size = processed_cloud->points.size();

        ROS_INFO("========================================");
        ROS_INFO("Starting Point Cloud Processing Pipeline");
        ROS_INFO("========================================");

        // 步骤1: XYZ三轴范围过滤
        if (use_range_filter_)
        {
            ROS_INFO("[Step 1] XYZ Range Filtering");
            processed_cloud = applyRangeFilter(processed_cloud);
            if (!processed_cloud || processed_cloud->empty())
            {
                ROS_ERROR("Range filtering resulted in empty point cloud!");
                return pcl::PointCloud<pcl::PointXYZ>::Ptr();
            }
        }

        // 步骤2: 离群点移除（可选）
        if (use_outlier_removal_)
        {
            ROS_INFO("[Step 2] Outlier Removal");
            processed_cloud = applyOutlierRemoval(processed_cloud);
            if (!processed_cloud || processed_cloud->empty())
            {
                ROS_ERROR("Outlier removal resulted in empty point cloud!");
                return pcl::PointCloud<pcl::PointXYZ>::Ptr();
            }
        }

        // 步骤3: 降采样
        if (use_downsampling_)
        {
            ROS_INFO("[Step 3] Downsampling");
            processed_cloud = downsampleCloud(processed_cloud);
            if (!processed_cloud || processed_cloud->empty())
            {
                ROS_ERROR("Downsampling resulted in empty point cloud!");
                return pcl::PointCloud<pcl::PointXYZ>::Ptr();
            }
        }

        // 显示总体统计信息
        if (display_filter_stats_)
        {
            size_t final_size = processed_cloud->points.size();
            float overall_reduction = 100.0f * (1.0f - static_cast<float>(final_size) / original_size);

            ROS_INFO("----------------------------------------");
            ROS_INFO("Processing Summary");
            ROS_INFO("----------------------------------------");
            ROS_INFO("Original points: %ld", original_size);
            ROS_INFO("Final points:    %ld", final_size);
            ROS_INFO("Total reduction: %.1f%%", overall_reduction);
            ROS_INFO("========================================");
        }

        return processed_cloud;
    }

    sensor_msgs::PointCloud2::Ptr convertToROSMsg(const pcl::PointCloud<pcl::PointXYZ>::Ptr & cloud)
    {
        if (!cloud || cloud->empty())
        {
            ROS_WARN("Point cloud data is empty");
            return sensor_msgs::PointCloud2::Ptr();
        }

        sensor_msgs::PointCloud2::Ptr ros_cloud(new sensor_msgs::PointCloud2);
        pcl::toROSMsg(*cloud, *ros_cloud);
        ros_cloud->header.frame_id = frame_id_;
        ros_cloud->header.stamp = ros::Time::now();

        return ros_cloud;
    }

    void run()
    {
        // 加载点云数据
        auto cloud = loadPCD();
        if (!cloud || cloud->empty())
        {
            ROS_ERROR("Cannot load PCD file");
            return;
        }

        // 处理点云：XYZ范围过滤 -> 离群点移除 -> 降采样
        auto processed_cloud = processPointCloud(cloud);

        // 检查处理后的点云是否有效
        if (!processed_cloud || processed_cloud->empty())
        {
            ROS_ERROR("Processed point cloud is empty");
            return;
        }

        // 转换为ROS消息并发布
        auto ros_cloud = convertToROSMsg(processed_cloud);

        if (ros_cloud)
        {
            // 发布处理后的点云（只发布一次）
            cloud_pub_.publish(ros_cloud);
            ROS_INFO("========================================");
            ROS_INFO("Publishing Point Cloud");
            ROS_INFO("========================================");
            ROS_INFO("Topic: %s", cloud_pub_.getTopic().c_str());
            ROS_INFO("Frame: %s", frame_id_.c_str());
            ROS_INFO("Points: %ld", processed_cloud->size());
            ROS_INFO("Timestamp: %.6f", ros_cloud->header.stamp.toSec());
            ROS_INFO("========================================");
            ROS_INFO("PCD processing completed successfully!");
        }
        else
        {
            ROS_ERROR("Failed to convert point cloud to ROS message");
        }

        // 保持节点运行，直到手动关闭
        ROS_INFO("Press Ctrl+C to exit...");
        ros::spin();
    }
};

int main(int argc, char ** argv)
{
    // 初始化ROS节点
    ros::init(argc, argv, "pcd_publisher");

    // 创建发布器对象并运行
    PCDPublisher publisher;
    publisher.run();

    return 0;
}