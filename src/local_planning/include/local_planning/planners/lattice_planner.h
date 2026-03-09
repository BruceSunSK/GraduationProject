// lattice_planner.h
#pragma once
#include <vector>
#include <array>
#include <memory>
#include <chrono>
#include "local_planning/planners/local_planner_interface.h"


class LatticePlanner : public LocalPlannerInterface
{
public:
    // Lattice 规划器参数结构体
    struct LatticePlannerParams : public LocalPlannerInterface::LocalPlannerParamsInterface
    {
        // 采样参数
        double sample_s_step = 2.0;             // 纵向采样步长 (m)
        double sample_l_range = 3.0;            // 横向采样范围 (±m)
        double sample_l_step = 0.2;             // 横向采样步长 (m)
        double sample_T_min = 3.0;              // 最小规划时间 (s)
        double sample_T_max = 7.0;              // 最大规划时间 (s)
        double sample_T_step = 1.0;             // 时间采样步长 (s)

        // 车辆参数
        double vehicle_length = 3.8;
        double vehicle_width = 2.0;
        double max_speed = 4.0;
        double max_accel = 3.0;
        double max_decel = -2.0;
        double max_curvature = 1.0;
        double max_lateral_accel = 2.0;

        // 参考速度
        double target_speed = 4.0;

        // 障碍物参数
        double collision_safety_margin = -1.5;   // 动态碰撞安全距离 (m)

        // 评价权重
        double weight_lateral_offset = 1.0;
        double weight_speed_deviation = 1.0;
        double weight_lateral_accel = 1.0;
        double weight_longitudinal_jerk = 1.0;
        double weight_obstacle = 1.0;
        double weight_travel_time = 0.1;
    };

private:
    // Frenet 状态
    struct FrenetState
    {
        double s;      // 纵向位置
        double s_d;    // 纵向速度
        double s_dd;   // 纵向加速度
        double l;      // 横向偏移
        double l_d;    // 横向速度 (dl/ds)
        double l_dd;   // 横向加速度 (d²l/ds²)
    };

    // 候选轨迹
    struct Trajectory
    {
        std::vector<Path::TrajectoryPoint> points;
        double cost;
    };

    // 采样终点
    struct SampleEndState
    {
        double s_target;   // 目标纵向位置
        double l_target;   // 目标横向偏移
        double v_target;   // 目标速度
        double T;          // 规划时间
    };

public:
    LatticePlanner() {};
    LatticePlanner(const LatticePlanner & other) = delete;
    LatticePlanner & operator=(const LatticePlanner & other) = delete;
    LatticePlanner(LatticePlanner && other) = delete;
    LatticePlanner & operator=(LatticePlanner && other) = delete;
    ~LatticePlanner() override = default;

    // 初始化
    void InitParams(const LocalPlannerParamsInterface & params) override;

    // 设置数据
    void SetMap(const Map::MultiMap::Ptr & map) override { map_ = map; }
    void SetReferencePath(const Path::ReferencePath::Ptr & reference_path) override { reference_path_ = reference_path; }
    void SetVehicleState(const Vehicle::State::Ptr & vehicle_state) override { vehicle_state_ = vehicle_state; }
    void SetObstacles(const Obstacle::Obstacle::List & obstacles) override { obstacles_ = obstacles; }

    // 规划主函数
    bool Plan(LocalPlannerResult & result, std::string & error_msg) override;

private:
    // 生成所有候选轨迹
    std::vector<Trajectory> GenerateTrajectories(const FrenetState & start,
        const std::vector<SampleEndState> & samples);

    // 评价轨迹
    double EvaluateTrajectory(const Trajectory & traj, const FrenetState & start);

    // 碰撞检测（使用代价地图）
    bool CheckCollision(const Trajectory & traj) const;

    // 动态碰撞检测
    bool CheckDynamicCollision(const Trajectory & traj) const;

    // 多项式系数计算
    std::array<double, 6> ComputeQuinticCoefficients(double x0, double dx0, double ddx0,
        double x1, double dx1, double ddx1, double T);
    std::array<double, 5> ComputeQuarticCoefficients(double x0, double dx0, double ddx0,
        double x1, double dx1, double T);

private:
    LatticePlannerParams params_;
    Map::MultiMap::Ptr map_;
    Path::ReferencePath::Ptr reference_path_;
    Vehicle::State::Ptr vehicle_state_;
    Obstacle::Obstacle::List obstacles_;
};
