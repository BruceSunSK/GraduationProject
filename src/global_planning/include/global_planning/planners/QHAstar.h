#pragma once
#include <queue>
#include <algorithm>

#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>

#include "global_planning/planners/global_planner_interface.h"
#include "global_planning/tools/print_struct_and_enum.h"
#include "global_planning/smoothers/discrete_point_smoother.h"


/// @brief 2023年IEEE IV的一篇论文，作者是清华大学的学生。https://ieeexplore.ieee.org/document/10186797
/// 我用来做对比实验，因此复现的过程中很多地方都没有很好的封装。
class QHAstar : public GlobalPlannerInterface
{
public:
    struct QHAstarParams : public GlobalPlannerParams
    {
        ~QHAstarParams() = default;

        struct
        {
            double WEIGTH_SMOOTH = 2.0;
            double WEIGTH_LENGTH = 1.0;
            double WEIGTH_DEVIATION = 1.0;
            REGISTER_STRUCT(REGISTER_MEMBER(WEIGTH_SMOOTH),
                            REGISTER_MEMBER(WEIGTH_LENGTH),
                            REGISTER_MEMBER(WEIGTH_DEVIATION));
        } smoothing;

        REGISTER_STRUCT(REGISTER_MEMBER(smoothing));
    };

    class QHAstarHelper : public GlobalPlannerHelper
    {
    public:
        using GlobalPlannerHelper::GlobalPlannerHelper;
        ~QHAstarHelper() = default;

        struct 
        {
            double cost_time = 0;           // 总耗时，单位ms
            REGISTER_STRUCT(REGISTER_MEMBER(cost_time))
        } path_planning;
            
        struct
        {
            double cost_time = 0;           // 总耗时，单位ms
            REGISTER_STRUCT(REGISTER_MEMBER(cost_time))
        } path_smoothing;

    public:
        /// @brief 清空当前记录的所有结果信息，便于下次记录
        void resetResultInfo() override
        {
            path_planning.cost_time = 0.0;
            path_smoothing.cost_time = 0.0;
        }

    private:
        std::string paramsInfo() const override;
        std::string mapInfo() const override;
        std::string resultInfo() const override;
    };

private:
    struct Node
    {
        enum class NodeType : uint8_t
        {
            UNKNOWN,
            CLOSED,
            OPENED
        };

        cv::Point2i point;  // 栅格的xy值
        double cost = 0;    // 栅格中的代价值

        double g = 0;       // 起点到该点已探索的可通行距离代价值
        double h = 0;       // 节点种类，标识是否已探索
        double f = 0;       // 该点总共的代价值 f = g + h
        NodeType type = NodeType::UNKNOWN;
        Node * parent_node = nullptr;   // 该节点的父节点

        /// @brief 节点比较重载，用于优先队列
        bool operator<(const Node & other) const { return f > other.f; }
        class NodePointerCmp
        {
        public:
            bool operator()(const Node * const l, const Node * const r) const { return l->f > r->f; }
        };
    };

public:
    QHAstar() : helper_(this) { planner_name_ = "QHAstar"; }
    QHAstar(const QHAstar & other) = delete;
    QHAstar(QHAstar && other) = delete;
    QHAstar & operator=(const QHAstar & other) = delete;
    QHAstar & operator=(QHAstar && other) = delete;
    ~QHAstar()= default;

private:
    /// @brief 对规划器相关变量进行初始化设置，进行参数拷贝设置
    /// @param params 传入的参数
    void initParams(const GlobalPlannerParams & params) override;
    /// @brief QH: 输入原始地图，然后设置成为规划器中需要使用的地图
    /// @param map 输入的原始地图
    /// @return 地图设置是否成功
    bool setMap(const cv::Mat & map) override;
    /// @brief 设置规划路径的起点。以真实地图坐标形式，而非行列形式。
    /// @param x 真实地图坐标系的x值
    /// @param y 真实地图坐标系的y值
    /// @return 该点是否能够成为起点。即该点在地图内部且不在障碍物上。
    bool setStartPoint(const double x, const double y) override;
    /// @brief 设置规划路径的起点。以真实地图坐标形式，而非行列形式。
    /// @param p 真实地图坐标系的点
    /// @return 该点是否能够成为起点。即该点在地图内部且不在障碍物上。
    bool setStartPoint(const cv::Point2d & p) override;
    /// @brief 设置规划路径的终点。以真实地图坐标形式，而非行列形式。
    /// @param x 真实地图坐标系的x值
    /// @param y 真实地图坐标系的y值
    /// @return 该点是否能够成为终点。即该点在地图内部且不在障碍物上。
    bool setEndPoint(const double x, const double y) override;
    /// @brief 设置规划路径的终点。以真实地图坐标形式，而非行列形式。
    /// @param p 真实地图坐标系的点
    /// @return 该点是否能够成为终点。即该点在地图内部且不在障碍物上。
    bool setEndPoint(const cv::Point2d & p) override;
    /// @brief 获得处理后的地图，即算法内部真正使用的，经过二值化后的地图
    /// @param map 地图将存入该变量
    /// @return 存入是否成功
    bool getProcessedMap(cv::Mat & map) const override;
    /// @brief 通过给定的地图、起点、终点规划出一条从起点到终点的最终路径。
    /// @param path 规划出的路径。该路径是原始地图坐标系下原始路径点。
    /// @param auxiliary_info 辅助信息。该信息是原始地图坐标系下路径规划过程中的各种关键路径点信息。此处包括AStar按顺序扩展到的节点。
    /// @return 是否规划成功
    bool getPath(std::vector<cv::Point2d> & path, std::vector<std::vector<cv::Point2d>> & auxiliary_info) override;
    /// @brief 打印所有的信息，包括规划器参数信息、规划地图信息、规划结果信息，并可以将结果保存到指定路径中。调用内部辅助helper实现
    /// @param save 是否保存到本地
    /// @param save_dir_path 保存的路径
    void showAllInfo(const bool save = false, const std::string & save_dir_path = "") const override { helper_.showAllInfo(save, save_dir_path); }
    /// @brief 获取所有信息的helper，用于记录规划耗时等信息。
    /// @note 该函数返回的是一个指针，存储的数据将会随着规划而刷新数据。
    /// @return 所有信息的helper
    const GlobalPlannerHelper * getAllInfo() const override { return static_cast<const GlobalPlannerHelper *>(&helper_); }

private:
    QHAstarParams params_;                  // 规划器参数
    QHAstarHelper helper_;                  // 规划器辅助

    std::vector<std::vector<Node>> map_;    // 实际使用地图
    cv::Mat distance_map_;                  // 距离障碍物的距离图
    double max_dis_ = 0;                    // distance_map_中的最大值
    double min_dis_ = std::numeric_limits<double>::max();   // distance_map_中的最小值
    Node * start_node_ = nullptr;           // 起点节点
    Node * end_node_   = nullptr;           // 终点节点

    /// @brief 根据启发类型，得到节点n到终点的启发值
    /// @param n 待计算的节点，计算该节点到终点的启发值
    double getH(const cv::Point2i & p) const;

    bool PathPlanning(std::vector<cv::Point2d> & path);

    bool PathSmoothing(const std::vector<cv::Point2d> & raw_path, std::vector<cv::Point2d> & path);
};
