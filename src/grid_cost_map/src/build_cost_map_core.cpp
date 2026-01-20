#include "grid_cost_map/build_cost_map.h"


BuildCostMap::BuildCostMap(ros::NodeHandle & nh, bool & success)
: nh_(nh), globalMapFilterChain("grid_map::GridMap"), localMapFilterChain("grid_map::GridMap"), listener_(buffer_)
{   
    // 配置ros参数
    global_laser_cloud_topic_ = nh_.param("global_laser_cloud_topic", std::string("/global_laser_cloud_map"));
    local_laser_cloud_topic_  = nh_.param("local_laser_cloud_topic", std::string("/all_lidars_preprocessed_ros"));
    global_map_load_path_     = nh_.param("global_map_load_path", std::string(""));

    global_map_res_   = nh_.param("global_map_res_", 0.2);
    local_map_res_    = nh_.param("local_map_res_", 0.2);
    local_map_length_ = nh_.param("local_map_length_", 20);


    subGlobalLaserCloud = nh_.subscribe(global_laser_cloud_topic_, 1, &BuildCostMap::GlobalLaserCloudCallback, this);
    subLocalLaserCloud  = nh_.subscribe(local_laser_cloud_topic_, 2, &BuildCostMap::LocalLaserCloudCallback, this);
    pubGlobalGridMap = nh_.advertise<grid_map_msgs::GridMap>("global_grid_map", 1, true);
    pubLocalGridMap  = nh_.advertise<grid_map_msgs::GridMap>("local_grid_map", 10, true);
    pubGlobalOccupancyGridMap = nh_.advertise<nav_msgs::OccupancyGrid>("global_occupancy_grid_map", 1, true);
    pubLocalOccupancyGridMap  = nh_.advertise<nav_msgs::OccupancyGrid>("local_occupancy_grid_map", 10, true);

    global_map.setFrameId("map");
    global_map.add("elevation");
    global_map.add("elevation_low");
    global_map.add("elevation_high");
    global_map_finished = false;
    local_map.setFrameId("veh");
    local_map.setGeometry(grid_map::Length(local_map_length_, local_map_length_), local_map_res_);
    local_map.add("elevation_low");
    local_map.add("elevation_high");
    local_map.add("elevation");
    local_map.add("traversability");
    local_map.add("traversability_inflated");


    // 加载预先生成好的png作为全局代价地图
    if (!global_map_load_path_.empty())
    {
        cv::Mat map = cv::imread(global_map_load_path_, cv::IMREAD_GRAYSCALE);
        int rows = map.rows;
        int cols = map.cols;

        nav_msgs::OccupancyGrid msg;
        msg.header.frame_id = "/map";
        msg.header.stamp = ros::Time::now();
        msg.info.height = rows;
        msg.info.width = cols;
        msg.info.origin.position.x = 0.0;
        msg.info.origin.position.y = 0.0;
        msg.info.resolution = global_map_res_;
        msg.data.resize(rows * cols);
        for (size_t i = 0; i < rows; i++)
        {
            for (size_t j = 0; j < cols; j++)
            {
                msg.data[i * cols + j] = map.at<uchar>(i, j);
            }
        }
        pubGlobalOccupancyGridMap.publish(msg);
    }

    // 配置全局代价地图滤波器
    if (!globalMapFilterChain.configure("global_map_filters", nh_))
    {
        ROS_ERROR("[GridCostMap]: Could not configure the global filter chain!");
        success = false;
        return;
    }
    
    // 配置局部代价地图滤波器
    if (!localMapFilterChain.configure("local_map_filters", nh_))
    {
        ROS_ERROR("[GridCostMap]: Could not configure the local filter chain!");
        success = false;
        return;
    }

    // 点云初始化
    global_map_laser_cloud.reset(new pcl::PointCloud<PointType>);
    loacl_map_laser_cloud.reset(new pcl::PointCloud<PointType>);

    success = true;
}

BuildCostMap::~BuildCostMap()
{
}

/// @brief 处理全局点云数据，通常为加载的全局pcd点云，只执行一次
/// @param msg 全局地图点云msg消息指针
void BuildCostMap::GlobalLaserCloudCallback(const sensor_msgs::PointCloud2::Ptr & msg)
{
    // 初始化grid_map
    global_map_laser_cloud->clear();
    pcl::fromROSMsg(*msg, *global_map_laser_cloud);

    PointType min_value, max_value;
    pcl::getMinMax3D(*global_map_laser_cloud, min_value, max_value);

    grid_map::Position oriPoint;
    oriPoint.x() = (min_value.x + max_value.x) / 2;
    oriPoint.y() = (min_value.y + max_value.y) / 2;

    global_map.setTimestamp(msg->header.stamp.toNSec());
    global_map.setGeometry(grid_map::Length(max_value.x - min_value.x + 2, max_value.y - min_value.y + 2), global_map_res_, oriPoint);
    ROS_INFO("[GridCostMap]: Created global map with size %f x %f m (%i x %i cells).",
             global_map.getLength().x(), global_map.getLength().y(),
             global_map.getSize()(0), global_map.getSize()(1));


    // 高程图elevation
    // 通过某一个栅格内最高和最低的点云确定，没有点云的栅格值为nan
    ros::Time start = ros::Time::now();
    for (int i = 0; i < global_map_laser_cloud->points.size(); i++)
    {
        PointType p = global_map_laser_cloud->points[i];

        std::vector<grid_map::Position> neighbors;
        neighbors.emplace_back(0.0, 0.0);
        constexpr double angles[] = {
            0.0,
            M_PI_4,
            M_PI_2,
            3 * M_PI_4,
            M_PI,
            -3 * M_PI_4,
            -M_PI_2,
            -M_PI_4
        };
        constexpr double step_radius = 0.15;
        for (double theta : angles)
        {
            const double dx = step_radius * std::cos(theta); // X方向分量
            const double dy = step_radius * std::sin(theta); // Y方向分量
            neighbors.emplace_back(dx, dy); // 隐式转换为grid_map::Position
        }

        for (auto && dir : neighbors)
        {
            grid_map::Position pos = dir + grid_map::Position(p.x, p.y);
            if (isnan(global_map.atPosition("elevation_low", pos)) || global_map.atPosition("elevation_low", pos) > p.z)
            {
                global_map.atPosition("elevation_low", pos) = p.z;
            }
            if (isnan(global_map.atPosition("elevation_high", pos)) || global_map.atPosition("elevation_high", pos) < p.z)
            {
                global_map.atPosition("elevation_high", pos) = p.z;
            }
        }
    }
    for (grid_map::GridMapIterator it(global_map); !it.isPastEnd(); ++it)
    {
        if (!isnan(global_map.at("elevation_low", *it)))
        {
            global_map.at("elevation", *it) = global_map.at("elevation_high", *it) - global_map.at("elevation_low", *it);
        }
    }
    // global_map.erase("elevation_low");
    // global_map.erase("elevation_high");
    ros::Time end = ros::Time::now();
    ROS_INFO("[GridCostMap]: elevation finished!, uesed %f s", (end - start).toSec());


    // 使用yaml中的滤波链进行滤波，得到可通行度traversability
    // 具体流程和内容如下：
    // 1. 对高程图使用中值滤波，填补空洞，得到“elevation_fill”图层
    // 2. 利用“elevation_fill”图层计算坡度，得到“slope”图层
    // 3. 用均值滤波使高程图光滑，得到“elevation_smooth”图层，然后计算滤波前后高程差值的绝对值，得到粗糙度“roughness”图层
    // 4. 用坡度的方差表示边界代价，得到“edges”图层
    // 5. 由 坡度、粗糙度、边界代价 加权得到的可通行度“traversability”图层
    start = ros::Time::now();
    if (!globalMapFilterChain.update(global_map, global_map))
    {
        ROS_ERROR("[GridCostMap]: Could not update the global map filter chain!");
        return;
    }
    else
    {
        global_map.erase("normal_vectors_x");
        global_map.erase("normal_vectors_y");

        end = ros::Time::now();
        ROS_INFO("[GridCostMap]: filter chain finished!, uesd %f s", (end - start).toSec());
    }
    

    // 后续处理
    for (grid_map::GridMapIterator it(global_map); !it.isPastEnd(); ++it)
    {
        // 1. 中值滤波补洞后的高程图中，没有激光雷达点地方，认为是为未探明区域
        if (isnan(global_map.at("elevation_fill", *it)))
        {
            if (!isnan(global_map.at("edges", *it)))
                global_map.at("traversability", *it) = 0.3 * global_map.at("edges", *it);
            else
                global_map.at("traversability", *it) = NAN;
        }
    }

    // {
    //     // 获取地图尺寸
    //     const grid_map::Size size = global_map.getSize();
    //     const int rows = size(1);  // GridMap 的行索引
    //     const int cols = size(0);  // GridMap 的列索引

    //     // 创建 OpenCV 矩阵（单通道 float）
    //     cv::Mat elevationMat(rows, cols, CV_32FC1);

    //     // 遍历 GridMap 并填充数据
    //     const grid_map::Matrix & elevationData = global_map["elevation_fill"];
    //     for (int i = 0; i < rows; ++i)
    //     {
    //         for (int j = 0; j < cols; ++j)
    //         {
    //             // GridMap 使用 (行, 列) 索引，对应 OpenCV 的 (行, 列)
    //             elevationMat.at<float>(i, j) = elevationData(cols - 1 - j, rows - 1 - i);
    //         }
    //     }

    //     // 使用 FileStorage 保存数据
    //     cv::FileStorage fs("/home/brucesun/elevation.yaml", cv::FileStorage::WRITE);
    //     fs << "elevation" << elevationMat;
    //     fs.release();
    //     ROS_INFO("[GridCostMap]: save elevation.yaml");
    // }

    // grid_map转成对应ros message
    grid_map_msgs::GridMap grid_map_msg;
    grid_map::GridMapRosConverter::toMessage(global_map, grid_map_msg);
    pubGlobalGridMap.publish(grid_map_msg);

    nav_msgs::OccupancyGrid occupancy_grid_map_msg;
    grid_map::GridMapRosConverter::toOccupancyGrid(global_map, "traversability", 0, 1, occupancy_grid_map_msg);
    pubGlobalOccupancyGridMap.publish(occupancy_grid_map_msg);

    // std::string file_path = ros::package::getPath("grid_cost_map") + "/map/XG_map.png";
    // cv::imwrite(file_path, map);
    global_map_finished = true;
}

/// @brief 处理局部实时点云数据，生成局部代价地图
/// @param msg 实时点云msg消息指针
void BuildCostMap::LocalLaserCloudCallback(const sensor_msgs::PointCloud2::Ptr & msg)
{
    // 局部代价地图依赖于全局代价地图
    if (global_map_finished == false)
    {
        ROS_WARN_THROTTLE(2, "[GridCostMap]: failed to create local cost map, waiting for global cost map!");
        return;
    }
    local_map.setTimestamp(msg->header.stamp.toNSec());

    // ros::Time start = ros::Time::now();

    // 获得全局地图和局部地图的坐标变换
    geometry_msgs::TransformStamped tfs;
    try
    {
        tfs = buffer_.lookupTransform("map", "rslidar_link", ros::Time(0), ros::Duration(0.01));
    }
    catch(const tf2::TransformException& e)
    {
        ROS_ERROR_THROTTLE(1, "%s", e.what());
        return;
    }

    local_map.clearAll();
    local_map["traversability_inflated"].setZero();

    // // 获取局部代价地图在全局代价地图中的值
    // for (grid_map::GridMapIterator it(local_map); !it.isPastEnd(); ++it) 
    // {
    //     grid_map::Position pos_local;
    //     local_map.getPosition(*it, pos_local);

    //     geometry_msgs::PointStamped point_local;
    //     point_local.header.stamp = msg->header.stamp;
    //     point_local.header.frame_id = "rslidar_link";
    //     point_local.point.x = pos_local.x();
    //     point_local.point.y = pos_local.y();
    //     point_local.point.z = 0;

    //     geometry_msgs::PointStamped point_global;
    //     tf2::doTransform(point_local, point_global, tfs);

    //     grid_map::Position pos_global;
    //     pos_global.x() = point_global.point.x;
    //     pos_global.y() = point_global.point.y;

    //     // 判断一个点是否都处于静态图层和动态图层

    //     if (local_map.isInside(pos_local) && global_map.isInside(pos_global))
    //     {
    //         local_map.atPosition("traversability", pos_local) = global_map.atPosition("traversability", pos_global);
    //     }
    // }


    // 根据实时点云计算代价值
    loacl_map_laser_cloud->clear();
    pcl::fromROSMsg(*msg, *loacl_map_laser_cloud);
    for (int i = 0; i < loacl_map_laser_cloud->points.size(); i++)
    {
        PointType p = loacl_map_laser_cloud->points[i];
        grid_map::Position pos;
        pos.x() = p.x;
        pos.y() = p.y;

        if (local_map.isInside(pos))
        {
            if (local_map.atPosition("elevation_low", pos) > p.z || isnan(local_map.atPosition("elevation_low", pos)))
            {
                local_map.atPosition("elevation_low", pos) = p.z;
            }
            if (local_map.atPosition("elevation_high", pos) < p.z || isnan(local_map.atPosition("elevation_high", pos)))
            {
                local_map.atPosition("elevation_high", pos) = p.z;
            }
        }
    }
    for (grid_map::GridMapIterator it(local_map); !it.isPastEnd(); ++it)
    {
        double elevation = local_map.at("elevation_high", *it) - local_map.at("elevation_low", *it);
        if (isnan(elevation))
        {
            // grid_map::Position pos_local;
            // local_map.getPosition(*it, pos_local);

            // geometry_msgs::PointStamped point_local;
            // point_local.header.stamp = msg->header.stamp;
            // point_local.header.frame_id = "rslidar_link";
            // point_local.point.x = pos_local.x();
            // point_local.point.y = pos_local.y();
            // point_local.point.z = 0;

            // geometry_msgs::PointStamped point_global;
            // tf2::doTransform(point_local, point_global, tfs);

            // grid_map::Position pos_global;
            // pos_global.x() = point_global.point.x;
            // pos_global.y() = point_global.point.y;

            // // 此时保持可通行度保留为全局地图的可通行度
            // // local_map.at("elevation", *it) = global_map.atPosition("elevation", pos_global);
            // if (global_map.isInside(pos_global))
            //     local_map.at("traversability", *it) = global_map.atPosition("traversability", pos_global);
            // else
            //     local_map.at("traversability", *it) = 1;

            local_map.at("traversability", *it) = 0;
        }
        // else if (elevation < 0)
        // {
        //     local_map.at("elevation", *it) = 0;
        //     local_map.at("traversability", *it) = 0;
        //     // local_map.at("traversability", *it) = global_map.atPosition("traversability", pos_global);
        // }
        else
        {
            // 更新高程图，并根据高程图的值重新计算可通行度
            // 使用带有死区的二次函数进行代价值曲线的拟合
            local_map.at("elevation", *it) = elevation;
            constexpr double dead_area = 0.1;
            if (elevation < dead_area)
                local_map.at("traversability", *it) = 0;
            else
                local_map.at("traversability", *it) = -(1 / std::pow(dead_area - 1, 2)) * std::pow(elevation - 1, 2) + 1;
        }
    }
    if (!localMapFilterChain.update(local_map, local_map))
    {
        ROS_ERROR("Could not update the local map filter chain!");
        return;
    }

    // 使用圆形区域对局部代价地图进行膨胀
    for (grid_map::GridMapIterator it(local_map); !it.isPastEnd(); ++it)
    {
        float trav = local_map.at("traversability", *it);
        grid_map::Position pos;
        local_map.getPosition(*it, pos);
        for (grid_map::CircleIterator it__(local_map, pos, 2 * local_map_res_); !it__.isPastEnd(); ++it__)
        {
            grid_map::Position pos_temp;
            local_map.getPosition(*it__, pos_temp);
            if (local_map.isInside(pos_temp))
            {
                if (local_map.at("traversability_inflated", *it__) < trav)
                    local_map.at("traversability_inflated", *it__) = trav;
            }
        }
    }

    // ros::Time end = ros::Time::now();
    // ROS_INFO("local map finished!, uesed %f s", (end - start).toSec());

    // 发布消息
    grid_map_msgs::GridMap grid_map_msg;
    grid_map::GridMapRosConverter::toMessage(local_map, grid_map_msg);
    pubLocalGridMap.publish(grid_map_msg);

    nav_msgs::OccupancyGrid occupancy_grid_map_msg;
    grid_map::GridMapRosConverter::toOccupancyGrid(local_map, "traversability_inflated", 0, 1, occupancy_grid_map_msg);
    pubLocalOccupancyGridMap.publish(occupancy_grid_map_msg);
}
