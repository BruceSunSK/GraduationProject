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
/// [2] 代价函数 f = g + w * h
///      1. h和原版AStar保持一致，为距离启发代价。此处选择Euclidean距离启发。
///      2. g为考虑可通行度的已探索距离代价，在考虑距离时也考虑可通行度代价。
///         具体实现为：根据栅格代价值计算出一个大于1的系数与原有的g相乘，实现考虑可通行度的距离代价。
///         cost为栅格的可通行代价；g0为距离代价，等价于原版astar中的g
///         g = \sigma gi = \sigma exp(k * costi / 100) * g0i
///         其中，k为自定义系数，k值越大，则远离障碍的趋势越明显，但也会导致出现绕弯。目前k = 2
///      3. [暂未使用] w为加权系数，此处选择为动态加权。
///         引入障碍物密度的概念P(p1, p2) = (\sigma cost) / (100 * (|p1.x - p2.x| + 1) * (|p1.y - p2.y| + 1)), \sigma cost为矩形区域内的总代价值。
///         然后，w = (1 - lnP), P ∈ (0, 1), w ∈ (1, ∞)。
///         可以实现在障碍物密集的区域实现避免搜索步长过大，出现局部最优，有效避开障碍物；在障碍物较少的区域中，加快搜索，减少搜索栅格个数。
/// [3] 去除冗余点
///      1. 在搜索过程中使用Direction枚举记录节点的扩展方向（即子节点位于父节点的方向）。
///      2. 在冗余点剔除时，若连续两个点的扩展方向相同，则剔除前一个节点。
/// [4] 引入贝塞尔曲线进行分段平滑。
///      1. 首先将路径点去除起点和终点后每两个划分为1组。第一组额外包括起点，最后一组额外包括终点
///      2. 然后第i组的第二个点与第i+1组的第一个点线性插值的中点作为新插入节点，同时作为第i组和第i+1组的成员。
///      3. 这样每组都拥有四个节点成员，使用三阶贝塞尔曲线进行平滑。这样在分段处基本保持连续和曲率平滑。
///      4. 最后一组可能由于点的数量不够，只有三个点，此时使用二阶贝塞尔曲线进行平滑。
/// [5] 由于生成的贝塞尔曲线是稠密的，因此手动进行降采样。
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
        /// @brief 表明该节点相对于父节点的方向。使用OpenCV坐标系
        enum class Direction : uint8_t
        {
            UNKNOWN = 0x00,
            E = 0x01,
            W = 0x02,
            N = 0x04,
            S = 0x08,
            NE = N | E | 0x10,
            NW = N | W | 0x10,
            SE = S | E | 0x10,
            SW = S | W | 0x10
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
        cv::Point2i point;  // 栅格的xy值
        double cost = 0;    // 栅格中的代价值
        double g = 0;       // 起点到该点已探索的可通行距离代价值
        double h = 0;       // 该点到终点的启发值
        double w = 0;       // 该点的权重值，为动态加权
        double w_cost = 0;  // 该点到终点矩形区域内代价值的和，为计算权重的中间量
        double f = 0;       // 该点总共的代价值 f = g + w * h
        NodeType type = NodeType::UNKNOWN;  // 节点种类，标识是否已探索
        Node * parent_node = nullptr;       // 该节点的父节点
        Direction direction_to_parent = Direction::UNKNOWN;     // 该节点相对父节点的方位


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
        Euclidean,
        Chebyshev,
        Octile
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
    bool getSmoothPath(std::vector<cv::Point2d> & path) override;

private:
    std::vector<std::vector<Node>> map_;
    Node * start_node_;
    Node * end_node_;

    HeuristicsType type_;

    BezierCurve bezier_;

    void getG(Node * n);
    void getH(Node * n);
    void getW(Node * n);
    bool generateRawNodes(std::vector<Node *> & raw_nodes);
    bool removeRedundantNodes(const std::vector<Node *> & raw_nodes, std::vector<Node *> & reduced_nodes);
    void nodesToPath(const std::vector<Node *> & nodes, std::vector<cv::Point2i> & path);
    bool smoothPath(const std::vector<cv::Point2i> & reduced_path, std::vector<cv::Point2d> & smooth_path);
    void resetMap();
};
