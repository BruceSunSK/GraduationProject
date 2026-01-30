#pragma once
#include <vector>
#include <string>

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
        const Vehicle::VehicleState & vehicle_state);


private:
    LocalPlannerParams params_;

    Vehicle::VehicleState vehicle_state_;


    std::pair<double, double> BackwardAndForwardDistance(
        const Path::ReferencePath::Ptr & reference_path) const;
};
