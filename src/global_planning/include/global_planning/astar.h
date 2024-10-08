#pragma once
#include <queue>
#include <algorithm>

#include "global_planning/global_planner_interface.h"
#include "global_planning/tools/print_struct_and_enum.h"


/// @brief 使用Astar算法的规划结果，无任何改进
class Astar : public GlobalPlannerInterface
{
public:
    /// @brief 搜索过程的启发值类型
    enum class HeuristicsType : uint8_t
    {
        None, 
        Manhattan, 
        Euclidean,
        Chebyshev,
        Octile
    };

    /// @brief 用于Astar规划器使用的参数类型
    struct AstarParams : public GlobalPlannerParams
    {
        ~AstarParams() = default;

        // 地图相关参数
        struct 
        {
            uint8_t OBSTACLE_THRESHOLD = 50;       // 地图中栅格代价值大于等于(>=)该值的栅格，会被视为障碍物，搜索过程中将直接跳过该栅格
            REGISTER_STRUCT(REGISTER_MEMBER(OBSTACLE_THRESHOLD));
        } map_params;

        // 代价函数相关参数
        struct
        {
            Astar::HeuristicsType HEURISTICS_TYPE = Astar::HeuristicsType::Euclidean;   // 启发值类型，共有五种
            REGISTER_STRUCT(REGISTER_MEMBER(HEURISTICS_TYPE));
        } cost_function_params;

        REGISTER_STRUCT(REGISTER_MEMBER(map_params),
                        REGISTER_MEMBER(cost_function_params));
    };

    /// @brief 用于Astar规划器的辅助类，实现数据记录和打印
    class AstarHelper : public GlobalPlannerHelper
    {
    public:
        using GlobalPlannerHelper::GlobalPlannerHelper;
        ~AstarHelper() = default;

        struct 
        {
            int node_nums = 0;          // 搜索到节点的数量，不包括障碍物节点
            int node_counter = 0;       // 搜索到节点的次数，会重复计算同一个节点，不包括障碍物节点
            double cost_time = 0;       // 搜索总耗时，单位ms

            REGISTER_STRUCT(REGISTER_MEMBER(node_nums),
                            REGISTER_MEMBER(node_counter),
                            REGISTER_MEMBER(cost_time))
        } search_result;

    public:
        /// @brief 打印所有的信息，包括规划器参数信息、规划地图信息、规划结果信息，并可以将结果保存到指定路径中。
        /// @param save 是否保存到本地
        /// @param save_dir_path 保存的路径
        void showAllInfo(const bool save = false, const std::string & save_dir_path = "") const override;
        /// @brief 清空当前记录的所有结果信息，便于下次记录
        void resetResultInfo() override
        {
            search_result.node_nums = 0;
            search_result.node_counter = 0;
            search_result.cost_time = 0;
        }

    private:
        /// @brief 将规划器中设置的参数以字符串的形式输出
        /// @return 规划器的参数
        std::string paramsInfo() const override;
        /// @brief 将规划器所使用的地图信息、起点、终点以字符串的形式输出
        /// @return 地图信息、起点、终点信息
        std::string mapInfo() const override;
        /// @brief 将helper中保存的所有有关规划的结果以字符串的形式输出
        /// @return 规划的结果数值
        std::string resultInfo() const override;
    };

private:
    /// @brief 用于描述规划过程中的单个栅格节点
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

        double g = 0;// 起点到该点已探索的可通行距离代价值
        // 该点到终点的启发值
        double h = 0;// 节点种类，标识是否已探索
        double f = 0;// 该点总共的代价值 f = g + w * h
        NodeType type = NodeType::UNKNOWN;
        Node * parent_node = nullptr;// 该节点的父节点

        /// @brief 节点比较重载，用于优先队列
        bool operator<(const Node & other) const { return f > other.f; }
        class NodePointerCmp
        {
        public:
            bool operator()(const Node * const l, const Node * const r) const { return l->f > r->f; }
        };
    };

public:
    Astar() : helper_(this) {}
    ~Astar() = default;

    /// @brief 对规划器相关变量进行初始化设置，进行参数拷贝设置
    /// @param params 传入的参数
    void initParams(const GlobalPlannerParams & params) override;
    /// @brief 对输入的map进行二值化，然后设置成为规划器中需要使用的地图
    /// @param map 输入的原始地图
    /// @return 地图设置是否成功
    bool setMap(const cv::Mat & map) override;
    /// @brief 设置规划路径的起点。以栅格坐标形式，而非行列形式。
    /// @param x 栅格坐标系的x值
    /// @param y 栅格坐标系的y值
    /// @return 该点是否能够成为起点。即该点在地图内部且不在障碍物上。
    bool setStartPoint(const int x, const int y) override;
    /// @brief 设置规划路径的起点。以栅格坐标形式，而非行列形式。
    /// @param p 栅格坐标系的点
    /// @return 该点是否能够成为起点。即该点在地图内部且不在障碍物上。
    bool setStartPoint(const cv::Point2i p) override;
    /// @brief 设置规划路径的终点。以栅格坐标形式，而非行列形式。
    /// @param x 栅格坐标系的x值
    /// @param y 栅格坐标系的y值
    /// @return 该点是否能够成为终点。即该点在地图内部且不在障碍物上。
    bool setEndPoint(const int x, const int y) override;
    /// @brief 设置规划路径的终点。以栅格坐标形式，而非行列形式。
    /// @param p 栅格坐标系的点
    /// @return 该点是否能够成为终点。即该点在地图内部且不在障碍物上。
    bool setEndPoint(const cv::Point2i p) override;
    /// @brief 获得处理后的地图，即算法内部真正使用的，经过二值化后的地图
    /// @param map 地图将存入该变量
    /// @return 存入是否成功
    bool getProcessedMap(cv::Mat & map) const override;
    /// @brief 通过给定的地图、起点、终点规划出一条从起点到终点的路径。
    /// @param path 规划出的路径。该路径是栅格坐标系下原始路径点。
    /// @return 是否规划成功
    bool getRawPath(std::vector<cv::Point2i> & path) override;
    /// @brief 通过给定的地图、起点、终点规划出一条从起点到终点的路径。目前并无平滑。
    /// @param path 规划出的路径。该路径是真实地图下的坐标点。此外目前无其他功能。
    /// @return 是否规划成功
    bool getSmoothPath(std::vector<cv::Point2d> & path) override;
    /// @brief 打印所有的信息，包括规划器参数信息、规划地图信息、规划结果信息，并可以将结果保存到指定路径中。调用内部辅助helper实现
    /// @param save 是否保存到本地
    /// @param save_dir_path 保存的路径
    void showAllInfo(const bool save, const std::string & save_dir_path) const override { helper_.showAllInfo(save, save_dir_path); }

private:
    AstarParams params_;                    // 规划器参数
    AstarHelper helper_;                    // 规划器辅助

    std::vector<std::vector<Node>> map_;    // 实际使用地图
    Node * start_node_ = nullptr;           // 起点节点
    Node * end_node_   = nullptr;           // 终点节点

    /// @brief 根据启发类型，得到节点n到终点的启发值
    /// @param n 待计算的节点，计算该节点到终点的启发值
    double getH(const cv::Point2i & p) const;
};

using HeuristicsType = Astar::HeuristicsType;
REGISTER_ENUM(HeuristicsType);
