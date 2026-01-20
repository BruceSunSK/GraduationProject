#include "grid_cost_map/build_cost_map.h"


BuildCostMap::BuildCostMap(ros::NodeHandle & nh, bool & success) 
: nh_(nh), globalMapFilterChain("grid_map::GridMap"), localMapFilterChain("grid_map::GridMap"), listener_(buffer_)
{   
    // 配置ros参数
    global_laser_cloud_topic_ = nh_.param("global_laser_cloud_topic", std::string("/global_laser_cloud_map"));
    local_laser_cloud_topic_ = nh_.param("local_laser_cloud_topic", std::string("/all_lidars_preprocessed_ros"));

    global_map_res_ = nh_.param("global_map_res_", 0.2);
    local_map_res_ = nh_.param("local_map_res_", 0.2);
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

    local_map.setFrameId("map");
    local_map.setGeometry(grid_map::Length(local_map_length_, local_map_length_), local_map_res_);
    local_map.add("elevation_low");
    local_map.add("elevation_high");
    local_map.add("elevation");
    local_map.add("traversability");

    local_map_out.setFrameId("veh");
    local_map_out.setGeometry(grid_map::Length(local_map_length_, local_map_length_), local_map_res_);
    local_map_out.add("traversability");

    // 配置滤波器
    if (!globalMapFilterChain.configure("global_map_filters", nh_))
    {
        ROS_ERROR("Could not configure the global filter chain!");
        success = false;
        return;
    }

    if (!localMapFilterChain.configure("local_map_filters", nh_))
    {
        ROS_ERROR("Could not configure the local filter chain!");
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
    ROS_INFO("Created global map with size %f x %f m (%i x %i cells).",
             global_map.getLength().x(), global_map.getLength().y(),
             global_map.getSize()(0), global_map.getSize()(1));


    // 高程图elevation
    // 通过某一个栅格内最高和最低的点云确定，没有点云的栅格值为nan
    ros::Time start = ros::Time::now();
    for (int i = 0; i < global_map_laser_cloud->points.size(); i++)
    {
        PointType p = global_map_laser_cloud->points[i];
        grid_map::Position pos;
        pos.x() = p.x;
        pos.y() = p.y;

        if (global_map.atPosition("elevation_low", pos) > p.z || isnan(global_map.atPosition("elevation_low", pos)))
        {
            global_map.atPosition("elevation_low", pos) = p.z;
        }
        if (global_map.atPosition("elevation_high", pos) < p.z || isnan(global_map.atPosition("elevation_high", pos)))
        {
            global_map.atPosition("elevation_high", pos) = p.z;
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
    ROS_INFO("elevation finished!, uesed %f s", (end - start).toSec());


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
        ROS_ERROR("Could not update the global map filter chain!");
        return;
    }
    else
    {
        global_map.erase("normal_vectors_x");
        global_map.erase("normal_vectors_y");

        end = ros::Time::now();
        ROS_INFO("filter chain finished!, uesd %f s", (end - start).toSec());
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

    // grid_map转成对应ros message
    grid_map_msgs::GridMap grid_map_msg;
    grid_map::GridMapRosConverter::toMessage(global_map, grid_map_msg);
    pubGlobalGridMap.publish(grid_map_msg);

    nav_msgs::OccupancyGrid occupancy_grid_map_msg;
    grid_map::GridMapRosConverter::toOccupancyGrid(global_map, "traversability", 0, 1, occupancy_grid_map_msg);
    pubGlobalOccupancyGridMap.publish(occupancy_grid_map_msg);

    global_map_finished = true;
}

void BuildCostMap::LocalLaserCloudCallback(const sensor_msgs::PointCloud2::Ptr & msg)
{
    // 局部代价地图依赖于全局代价地图
    if (global_map_finished == false)
    {
        ROS_WARN_THROTTLE(2, "failed to create local cost map, waiting for global cost map!");
        return;
    }
    local_map.setTimestamp(msg->header.stamp.toNSec());

    // 获得全局地图和局部地图的坐标变换
    geometry_msgs::TransformStamped tfs;
    try
    {
        tfs = buffer_.lookupTransform("map", "veh", ros::Time(0), ros::Duration(0.01));
    }
    catch(const tf2::TransformException& e)
    {
        ROS_ERROR_THROTTLE(1, "%s", e.what());
        return;
    }

    ros::Time start = ros::Time::now();

    static bool init = false;
    if (init == false)
    {
        // 设置局部代价地图初始位置
        local_map.setPosition(grid_map::Position(tfs.transform.translation.x, tfs.transform.translation.y));
        // 获取局部代价地图在全局代价地图中的值
        for (grid_map::GridMapIterator it(local_map); !it.isPastEnd(); ++it) 
        {
            grid_map::Position pos;
            local_map.getPosition(*it, pos);

            // 判断局部地图中的点也在全局地图内
            if (global_map.isInside(pos))
            {
                local_map.at("traversability", *it) = global_map.atPosition("traversability", pos);
                local_map.at("elevation_low", *it)  = global_map.atPosition("elevation_low", pos);
                // local_map.at("elevation_high", *it) = global_map.atPosition("elevation_high", pos);
                local_map.at("elevation", *it)      = global_map.atPosition("elevation", pos);
            }
        }
        init = true;
    }
    else
    {
        std::vector<grid_map::BufferRegion> newRegions;
        local_map.move(grid_map::Position(tfs.transform.translation.x, tfs.transform.translation.y), newRegions);

        // 遍历每个新增区域
        for (const auto& region : newRegions) 
        {
            grid_map::Index index = region.getStartIndex();
            grid_map::Size size = region.getSize();
            for (int i = 0; i < size(0); i++)
            {
                for (int j = 0; j < size(1); j++)
                {
                    grid_map::Index now_index = grid_map::Index(index(0) + i, index(1) + j);
                    grid_map::Position pos;
                    local_map.getPosition(now_index, pos);

                    // 判断局部地图中的点也在全局地图内
                    if (global_map.isInside(pos))
                    {
                        local_map.at("traversability", now_index) = global_map.atPosition("traversability", pos);
                        local_map.at("elevation_low", now_index)  = global_map.atPosition("elevation_low", pos);
                        // local_map.at("elevation_high", now_index) = global_map.atPosition("elevation_high", pos);
                        local_map.at("elevation", now_index)      = global_map.atPosition("elevation", pos);
                    }
                }
            }
        }
    }

    // 根据实时点云计算代价值
    boost::shared_ptr<pcl::PointCloud<PointType>> loacl_map_laser_cloud_in_veh(new pcl::PointCloud<PointType>);
    pcl::fromROSMsg(*msg, *loacl_map_laser_cloud_in_veh);
    const Eigen::Affine3d tf_affine = tf2::transformToEigen(tfs);
    const Eigen::Matrix4f tf_matrix = tf_affine.matrix().cast<float>();
    loacl_map_laser_cloud->clear();
    pcl::transformPointCloud(*loacl_map_laser_cloud_in_veh, *loacl_map_laser_cloud, tf_matrix);

    local_map.clear("elevation_high");
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
        grid_map::Position pos;
        local_map.getPosition(*it, pos);

        double elevation = local_map.at("elevation_high", *it) - local_map.at("elevation_low", *it);
        if (isnan(elevation))
        {
            // 此时保持可通行度保留为全局地图的可通行度
            // if (isnan(global_map.atPosition("traversability", pos)))
            local_map.at("elevation", *it) = 0;
        }
        else if (elevation < 0)
        {
            local_map.at("elevation", *it) = 0;
            local_map.at("traversability", *it) = 0;
            // local_map.at("traversability", *it) = global_map.atPosition("traversability", pos);
        }
        else
        {
            // 更新高程图，并根据高程图的值重新计算可通行度
            local_map.at("elevation", *it) = elevation;
            if (elevation < 0.3)
                local_map.at("traversability", *it) = 0;
            else if (elevation < 0.6)
                local_map.at("traversability", *it) = 2 * elevation - 0.6;
            else
                local_map.at("traversability", *it) = 1;
        }
    }
    if (!localMapFilterChain.update(local_map, local_map))
    {
        ROS_ERROR("Could not update the local map filter chain!");
        return;
    }

    local_map_out.clear("traversability");
    for (grid_map::GridMapIterator it(local_map_out); !it.isPastEnd(); ++it)
    {
        grid_map::Position pos_in_veh;
        local_map_out.getPosition(*it, pos_in_veh);

        geometry_msgs::PointStamped point_in_veh;
        point_in_veh.header.stamp = msg->header.stamp;
        point_in_veh.header.frame_id = "veh";
        point_in_veh.point.x = pos_in_veh.x();
        point_in_veh.point.y = pos_in_veh.y();

        geometry_msgs::PointStamped point_global;
        tf2::doTransform(point_in_veh, point_global, tfs);

        grid_map::Position pos_in_map;
        pos_in_map.x() = point_global.point.x;
        pos_in_map.y() = point_global.point.y;

        double trav = 0;
        if (local_map.isInside(pos_in_map))
            trav = local_map.atPosition("traversability", pos_in_map);
        else if (global_map.isInside(pos_in_map))
            trav = global_map.atPosition("traversability", pos_in_map);
        else
            trav = 1;

        for (grid_map::CircleIterator it__(local_map_out, pos_in_veh, 1.5*local_map_res_); !it__.isPastEnd(); ++it__)
        {
            grid_map::Position pos_temp;
            local_map_out.getPosition(*it__, pos_temp);
            if (local_map_out.isInside(pos_temp))
            {
                if (isnan(local_map_out.at("traversability", *it__)) || local_map_out.at("traversability", *it__) < trav)
                    local_map_out.at("traversability", *it__) = trav;
            }
        }
    }


    ros::Time end = ros::Time::now();
    ROS_INFO("local map finished!, uesed %f s", (end - start).toSec());

    // 发布消息
    grid_map_msgs::GridMap grid_map_msg;
    grid_map::GridMapRosConverter::toMessage(local_map_out, grid_map_msg);
    pubLocalGridMap.publish(grid_map_msg);

    nav_msgs::OccupancyGrid occupancy_grid_map_msg;
    grid_map::GridMapRosConverter::toOccupancyGrid(local_map_out, "traversability", 0, 1, occupancy_grid_map_msg);
    pubLocalOccupancyGridMap.publish(occupancy_grid_map_msg);
}
