#pragma once
#include <vector>
#include <string>

#include "global_planning/map/distance_map.h"
#include "global_planning/path/utils.h"
#include "global_planning/path/reference_path.h"
#include "local_planning/vehicle/data_type.h"


/// @brief 局部规划器
class LocalPlanner
{
public:
    /// @brief 局部规划器参数结构体
    struct LocalPlannerParams
    {
        struct
        {
            double S_INTERVAL = 0.5;   // m
            double TRUNCATED_BACKWARD_S = 5.0;  // m
            double TRUNCATED_FORWARD_S = 20.0;   // m
        } reference_path;

        struct
        {
            double LENGTH = 3.8; // m
            double WIDTH = 2.0; // m
            double COLLISION_CIRCLE_RADIUS = 1.183685;  // m，碰撞圆半径，车辆使用三碰撞圆检测时的半径。 =sqrt( (l/6)^2 + (w/2)^2 )
            double COLLISION_SAFETY_MARGIN = 0.1;  // m，冗余安全距离，在三碰撞圆和OBB碰撞检测时都使用
        } vehicle;

        struct
        {
            double BOUND_SEARCH_RANGE = 10.0;  // m， 在sl坐标系下，搜索上下边界时的单边范围
            double BOUND_SEARCH_LARGR_RESOLUTION = 0.5;  // m， 在sl坐标系下，搜索上下边界时的粗分辨率
            double BOUND_SEARCH_SMALL_RESOLUTION = 0.1;  // m， 在sl坐标系下，搜索上下边界时的细分辨率
        } map;
    };

public:
    LocalPlanner() = default;
    LocalPlanner(const LocalPlanner & other) = delete;
    LocalPlanner(LocalPlanner && other) = delete;
    LocalPlanner & operator=(const LocalPlanner & other) = delete;
    LocalPlanner & operator=(LocalPlanner && other) = delete;
    ~LocalPlanner() = default;


    /// @brief 对规划器相关变量进行初始化设置，进行参数拷贝设置
    /// @param params 传入的参数
    void InitParams(const LocalPlannerParams & params);
    /// @brief 执行规划主函数
    /// @param reference_path 全局参考线
    /// @param vehicle_state 车辆状态
    /// @param obstacles 障碍物列表
    /// @param trajectory 输出的局部轨迹
    /// @return 规划是否成功
    // bool Plan(const Path::ReferencePath::Ptr & reference_path,
    //     const TrajectoryPoint & vehicle_state,
    //     const std::vector<Obstacle> & obstacles,
    //     Trajectory & trajectory);
    bool Plan(const Path::ReferencePath::Ptr & reference_path,
        const Vehicle::VehicleState & vehicle_state,
        const Map::MultiMap & map);


private:
    LocalPlannerParams params_;

    Vehicle::VehicleState vehicle_state_;


    std::pair<double, double> GetBackwardAndForwardDistance(
        const Path::ReferencePath::Ptr & reference_path) const;
    std::vector<std::pair<double, double>> GetBoundsByMap(
        const Path::ReferencePath::Ptr & reference_path,
        const Map::MultiMap & map) const;
};
