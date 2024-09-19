#pragma once
#include <iostream>
#include <vector>
#include <queue>
#include <unordered_map>
#include <algorithm>

#include "my_astar/global_planning_interface.h"
#include "my_astar/bezier_curve.h"

/// @brief Multi-layered Costmap Astar
/// [1] 实现栅格代价值离散化，0~100和255。代价值低处更易通过；100为障碍物，彻底不可通过；255为未探索区域（与建图相关），直接当做障碍物处理。
/// [2] 代价函数 f = g + h
///      1. h和原版AStar保持一致，为距离启发代价
///      2. g为考虑可通行度的已探索距离代价，在考虑距离的基础上外加可通行度代价作为衡量指标。
///         cost为栅格可通行代价；g0为距离代价，等价于原版astar中的g
///         g = \sigma gi = \sigma exp(cost / 100) * g0i
/// [3] 去除冗余点
/// [4] 引入贝塞尔曲线进行分段平滑。默认是每10个控制点进行一组贝塞尔曲线平滑，多组平滑结果拼接得到最终曲线。
class MCAstar : public GlobalPlannerInterface
{
    /// @brief 用于描述规划过程中的单个栅格节点
    struct Node
    {
        // 相关配置
        /// @brief 节点属性
        enum class NodeType : uint8_t
        {
            UNKNOWN,
            CLOSED,
            OPENED
        };
        /// @brief 表明该节点相对于父节点的方向
        enum class Direction : uint8_t
        {
            UNKNOWN,
            N,  S,  W,  E,
            NW, NE, SW, SE
        };
        /// @brief stl的std::unordered_map不支持std::pair的映射...自己写对应的hash的仿函数
        struct HashPair
        {
            template<typename T, typename U>
            size_t operator()(const std::pair<T, U> & p) const { return std::hash<T>()(p.first) ^ std::hash<U>()(p.second); }
        };
        /// @brief 同上，用于std::pair的键值比较，哈希碰撞的比较定义，需要直到两个自定义对象是否相等
        struct EqualPair 
        {
            template<typename T, typename U>
            bool operator ()(const std::pair<T, U> & p1, const std::pair<T, U> & p2) const { return p1.first == p2.first && p1.second == p2.second; }
        };
        /// @brief 索引-枚举的映射
        static const std::unordered_map<std::pair<int, int>, Direction, HashPair, EqualPair> index_direction_map;


        /// @brief 节点属性
        cv::Point2i point;
        float cost = 0;
        float g = 0;
        float h = 0;
        float f = 0;
        NodeType type = NodeType::UNKNOWN;
        Node * parent_node = nullptr;
        Direction direction_to_parent = Direction::UNKNOWN;
        bool is_redundant = false;


        /// @brief 节点比较重载，用于优先队列
        bool operator<(const Node & other) const { return f > other.f; }
        struct NodePointerCmp
        {
            bool operator()(Node * l, Node * r) const { return l->f > r->f; }
        };
    };

    /// @brief 搜索过程的启发值类型
    enum class HeuristicsType
    {
        None, 
        Manhattan, 
        Euclidean
    };
    
public:
    MCAstar(HeuristicsType type = HeuristicsType::Euclidean);
    ~MCAstar();

    bool setMap(const cv::Mat & map) override;
    bool setStartPoint(const int x, const int y) override;
    bool setStartPoint(const cv::Point2i p) override;
    bool setEndPoint(const int x, const int y) override;
    bool setEndPoint(const cv::Point2i p) override;

    bool getRawPath(std::vector<cv::Point2i> & path) override;
    bool getSmoothPath(std::vector<cv::Point2f> & path) override;

private:
    std::vector<std::vector<Node>> map_;
    Node * start_node_;
    Node * end_node_;

    int init_map_;
    int init_start_node_;
    int init_end_node_;
    HeuristicsType type_;

    BezierCurve bezier_;

    float getH(cv::Point2i p);
    bool generateRawNodes(std::vector<Node *> & raw_nodes);
    bool removeRedundantNodes(const std::vector<Node *> & raw_nodes, std::vector<Node *> & reduced_nodes);
    void nodesToPath(const std::vector<Node *> & nodes, std::vector<cv::Point2i> & path);
    bool smoothPath(const std::vector<cv::Point2i> & reduced_path, std::vector<cv::Point2f> & smooth_path, int control_nums_per_subpath = 10);
};
