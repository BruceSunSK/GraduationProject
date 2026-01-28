#pragma once
#include <vector>
#include <string>


/// @brief 局部规划器
class LocalPlanner
{
public:
    /// @brief 局部规划器参数结构体
    struct LocalPlannerParams
    {
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
    /// @param global_reference 全局参考线
    /// @param vehicle_state 车辆状态
    /// @param obstacles 障碍物列表
    /// @param trajectory 输出的局部轨迹
    /// @return 规划是否成功
    bool Plan(const std::vector<TrajectoryPoint> & global_reference,
        const TrajectoryPoint & vehicle_state,
        const std::vector<Obstacle> & obstacles,
        Trajectory & trajectory);


private:
    LocalPlannerParams params_;
};
