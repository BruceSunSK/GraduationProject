// local_planner_interface.h
#pragma once
#include <vector>
#include <string>
#include <sstream>
#include <memory>
#include <array>

#include "global_planning/map/distance_map.h"
#include "global_planning/path/reference_path.h"
#include "local_planning/vehicle/data_type.h"
#include "local_planning/obstacles/obstacle.h"


// 通用规划结果结构体
struct LocalPlannerResult
{
    double timestamp = 0.0;
    double planning_cost_time = 0.0;
    std::vector<Path::TrajectoryPoint> trajectory;

    std::stringstream log;
    std::vector<std::array<Path::PointXY, 3>> path_qp_lb;   // Lattice 无需，保留为空
    std::vector<std::array<Path::PointXY, 3>> path_qp_ub;   // Lattice 无需，保留为空

    void Clear()
    {
        timestamp = 0.0;
        planning_cost_time = 0.0;
        trajectory.clear();
        log.str("");
        path_qp_lb.clear();
        path_qp_ub.clear();
    }
};

// 局部规划器抽象基类
class LocalPlannerInterface
{
public:
    struct LocalPlannerParamsInterface
    {
        virtual ~LocalPlannerParamsInterface() = default;
    };

public:
    virtual ~LocalPlannerInterface() = default;

    // 初始化（具体参数由派生类自行管理）
    virtual void InitParams(const LocalPlannerParamsInterface & params) = 0;

    // 设置输入数据
    virtual void SetMap(const Map::MultiMap::Ptr& map) = 0;
    virtual void SetReferencePath(const Path::ReferencePath::Ptr& reference_path) = 0;
    virtual void SetVehicleState(const Vehicle::State::Ptr& vehicle_state) = 0;
    virtual void SetObstacles(const Obstacle::Obstacle::List& obstacles) = 0;

    // 执行规划，输出结果
    virtual bool Plan(LocalPlannerResult & result, std::string & error_msg) = 0;
};
