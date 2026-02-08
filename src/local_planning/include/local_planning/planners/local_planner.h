#pragma once
#include <vector>
#include <string>
#include <sstream>
#include <chrono>

#include "global_planning/map/distance_map.h"
#include "global_planning/path/utils.h"
#include "global_planning/path/reference_path.h"
#include "global_planning/smoothers/piecewise_jerk_smoother2.h"
#include "local_planning/vehicle/data_type.h"
#include "local_planning/vehicle/collision.h"
#include "local_planning/obstacles/obstacle.h"


/// @brief 局部规划器
class LocalPlanner
{
public:
    /// @brief 局部规划器参数结构体
    struct LocalPlannerParams
    {
        struct
        {
            double PLANNING_CYCLE_TIME = 0.1;  // s
        } common;

        struct
        {
            double S_INTERVAL = 0.5;   // m
            double TRUNCATED_BACKWARD_S = 5.0;  // m
            double TRUNCATED_FORWARD_S = 20.0;   // m
        } reference_path;

        struct
        {
            double V_TOLERANCE = 0.1;  // m/s
            double W_TOLERANCE = 0.001;  // rad/s
            double POS_TOLERANCE = 0.1;  // m
        } start_point;

        struct
        {
            // 几何参数
            double LENGTH = 3.8; // m
            double WIDTH = 2.0; // m
            double CENTER_TO_COLLISION_CENTER = LENGTH / 3; // m，车辆几何中心到前后碰撞圆中心的距离。 = l/3
            double COLLISION_CIRCLE_RADIUS = std::sqrt( LENGTH*LENGTH/36 + WIDTH*WIDTH/4 );
                                                            // m，碰撞圆半径，车辆使用三碰撞圆检测时的半径。 =sqrt( (l/6)^2 + (w/2)^2 )
            double COLLISION_SAFETY_MARGIN = 0.1;           // m，冗余安全距离，在三碰撞圆和OBB碰撞检测时都使用

            // 运动学参数
            double MAX_KAPPA = 0.5;                         // 1/m，车辆最大曲率。路径qp过程中l''约束用到。
        } vehicle;

        struct
        {
            double BOUND_SEARCH_RANGE = 10.0;  // m， 在sl坐标系下，搜索上下边界时的单边范围
            double BOUND_SEARCH_LARGE_RESOLUTION = 0.5;  // m， 在sl坐标系下，搜索上下边界时的粗分辨率
            double BOUND_SEARCH_SMALL_RESOLUTION = 0.1;  // m， 在sl坐标系下，搜索上下边界时的细分辨率
        } map;

        struct
        {
            double WEIGHT_L = 1.0;                      // 在路径qp过程中，路径点在参考线上L项的权重。
            double WEIGHT_DL = 50.0;                   // 在路径qp过程中，路径点在参考线上L'项的权重。
            double WEIGHT_DDL = 1000.0;                 // 在路径qp过程中，路径点在参考线上L''项的权重。
            double WEIGHT_DDDL = 1000.0;                // 在路径qp过程中，路径点在参考线上L'''项的权重。
            double WEIGHT_END_STATE_L = 1.0;           // 在路径qp过程中，靠近给定终点L项的权重。
            double WEIGHT_END_STATE_DL = 5.0;          // 在路径qp过程中，靠近给定终点L'项的权重。
            double WEIGHT_END_STATE_DDL = 50.0;        // 在路径qp过程中，靠近给定终点L''项的权重。
            double DL_LIMIT = 2.0;                      // 在路径qp过程中，l'绝对值的最大值约束。
        } path_qp;
    };

    struct LocalPlannerResult
    {
        // 规划结果
        double timestamp = 0.0;                             // 规划时间戳，单位：秒
        double planning_cost_time = 0.0;                         // 规划耗时，单位：秒
        std::vector<Path::TrajectoryPoint> trajectory;

        // 附加信息
        std::stringstream log;
        std::vector<std::array<Path::PointXY, 3>> path_qp_lb;
        std::vector<std::array<Path::PointXY, 3>> path_qp_ub;

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

public:
    LocalPlanner() : last_veh_proj_nearest_idx_(0) {};
    LocalPlanner(const LocalPlanner & other) = delete;
    LocalPlanner(LocalPlanner && other) = delete;
    LocalPlanner & operator=(const LocalPlanner & other) = delete;
    LocalPlanner & operator=(LocalPlanner && other) = delete;
    ~LocalPlanner() = default;

    void InitParams(const LocalPlannerParams & params);
    void SetMap(const Map::MultiMap::Ptr & map);
    void SetReferencePath(const Path::ReferencePath::Ptr & reference_path);
    void SetVehicleState(const Vehicle::State::Ptr & vehicle_state);
    bool Plan(LocalPlannerResult & result, std::string & error_msg);


private:
    LocalPlannerParams params_;
    LocalPlannerResult result_;
        
    // 存储外界数据 及 标志位
    Map::MultiMap::Ptr map_;
    Path::ReferencePath::Ptr reference_path_;
    Vehicle::State::Ptr vehicle_state_;
    Obstacle::Obstacle::List obstacles_;
    bool flag_map_ = false;
    bool flag_reference_path_ = false;
    bool flag_vehicle_state_ = false;
    bool has_obstacles_ = false;

    // 上帧规划结果
    bool has_last_trajectory_ = false;  // 是否有上一帧轨迹
    Path::PathNode last_start_point_;   // 上一帧的规划起点
    std::vector<Path::TrajectoryPoint> last_trajectory_;  // 上一帧轨迹
    std::chrono::steady_clock::time_point last_planning_time_;  // 上一帧规划时间
    int last_veh_proj_nearest_idx_;
    double last_planning_cycle_time_ = 0.1;  // 上一帧规划周期，默认0.1s

    // 本帧中间变量
    double curr_s_interval_;

    /// @brief 检查算法所需数据是否准备好
    /// @return 是否可以进行算法部分
    bool IsDataReady() const;
    /// @brief 匹配上一帧轨迹中的对应点
    /// @param current_pos 当前车辆位置
    /// @param matched_point 输出的匹配点
    /// @param lateral_error 横向误差
    /// @param longitudinal_error 纵向误差
    /// @return 是否成功匹配
    bool MatchLastTrajectory(const Path::PathNode & current_pos,
        Path::TrajectoryPoint & matched_point,
        double & lateral_error,
        double & longitudinal_error);
    /// @brief 使用运动学外推获取规划起点
    /// @param curr_veh_state 当前车辆状态
    /// @return 外推后的规划起点
    Path::PathNode GetMotionExtrapolationStart(const Vehicle::State & curr_veh_state);
    /// @brief 根据车辆当前速度信息、与上帧规划结果的信息，得到当前规划起点位置
    /// @param curr_veh_state 本帧车辆状态
    /// @param veh_proj_point 本帧车辆在参考线上的投影点
    /// @return 本帧规划起点位置
    Path::PathNode GetPlanningStart(const Vehicle::State & curr_veh_state, 
        const Path::PathNode & veh_proj_point);
    /// @brief 根据车辆规划起点位置，得到局部采样点，截取车辆周围的参考线，以PathNode的形式返回。
    /// @param planning_start_point 规划起点位置
    /// @return 车辆周围的参考线采样点，即局部采样点，也是本帧局部规划的范围
    std::vector<Path::PathNode> SampleReferencePoints(const Path::PathNode & planning_start_point);
    /// @brief 将碰撞圆圆心进行偏移，偏移后的位置是由参考线上的点投影而来
    /// @param original_node 车辆几何中心
    /// @param actual_node 当前碰撞圆圆心
    /// @param len 当前碰撞圆圆心到车辆几何中心的距离，有正负planning_start_point
    /// @return 偏移后的碰撞圆圆心
    Path::PathNode GetApproxNode(const Path::PathNode & original_node, 
        const Path::PathNode & actual_node, double len) const;
    /// @brief 根据代价地图计算当前所有碰撞圆的边界
    /// @param ref_points 局部采样点
    /// @return 所有碰撞圆的边界
    std::vector<std::array<std::pair<double, double>, 3>> GetBoundsByMap(
        const std::vector<Path::PathNode> & ref_points);
    /// @brief 进行路径QP优化，得到最优路径
    /// @param ref_points 局部采样点
    /// @param bounds 优化边界，包括碰撞圆边界与障碍物边界
    /// @param start_point 规划起点位置
    /// @param optimized_path 最优路径
    /// @return 是否成功进行路径QP优化
    bool PathPlanning(const std::vector<Path::PathNode> & ref_points,
        const std::vector<std::array<std::pair<double, double>, 3>> & bounds,
        const Path::PathNode & start_point,
        std::vector<Path::PointXY> & optimized_path);
};
