#pragma once
#include <iostream>
#include <vector>
#include <queue>
#include <unordered_map>
#include <algorithm>

#include "global_planning/global_planner_interface.h"
#include "global_planning/tools/bezier_curve.h"
#include "global_planning/tools/path_simplification.h"


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
///      1. DouglasPeucker法。以起点终点连线为基线，找到最远的点，超过threshold则只保留起点终点；否则以该点递归左右两侧。
///      2. DistanceThreshold法。以起点开始依次向该点之后连线，判断该点之后的点到该直线距离，超过阈值则保留该点，并重置上述过程；否则舍去，继续向后。
///      3. AngleThreshold法。逻辑和DistanceThreshold法一致，只不过判断方法变成了角度（<新点，原直线起点连线> 和 <原直线>的角度）。
/// [4] 引入贝塞尔曲线进行分段平滑。
///      1. 首先将路径点去除起点和终点后每两个划分为1组。第一组额外包括起点，最后一组额外包括终点
///      2. 然后第i组的第二个点与第i+1组的第一个点线性插值的中点作为新插入节点，同时作为第i组和第i+1组的成员。
///      3. 这样每组都拥有四个节点成员，使用三阶贝塞尔曲线进行平滑。这样在分段处基本保持连续和曲率平滑。
///      4. 最后一组可能由于点的数量不够，只有三个点，此时使用二阶贝塞尔曲线进行平滑。
/// [5] 由于生成的贝塞尔曲线是稠密的，因此手动进行降采样。按照两点间距离进行判断，使得路径离散。
class MCAstar : public GlobalPlannerInterface
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

    /// @brief 去除冗余点时使用的方法类型
    enum class PathSimplificationType : uint8_t
    {
        DouglasPeucker,
        DistanceThreshold,
        AngleThreshold
    };

    /// @brief 用于MCAstar规划器使用的参数类型
    struct MCAstarParams : public GlobalPlannerParams
    {
        ~MCAstarParams() = default;

        // 地图相关参数
        struct 
        {
            double EXPANDED_K = 1.0;                // 地图膨胀时，膨胀系数
            uint8_t EXPANDED_MIN_THRESHOLD = 0;     // 原始地图栅格代价值大于等于(>=)该值的栅格，才进行膨胀
            uint8_t EXPANDED_MAX_THRESHOLD = 100;   // 原始地图栅格代价值小于等于(<=)该值的栅格，才进行膨胀
            uint8_t COST_THRESHOLD = 10;            // 膨胀地图中栅格代价值大于等于(>=)该值的栅格，会被视为有代价值的栅格，否则代价值为0
            uint8_t OBSTACLE_THRESHOLD = 100;       // 膨胀地图中栅格代价值大于等于(>=)该值的栅格，会被视为障碍物，搜索过程中将直接跳过该栅格
        } map_params;

        // 代价函数相关参数
        struct
        {
            MCAstar::HeuristicsType HEURISTICS_TYPE = MCAstar::HeuristicsType::Euclidean;   // 启发值类型，共有五种
            double TRAV_COST_K = 2.0;   // 计算可通行度代价值时的系数
        } cost_function_params;

        // 冗余点去除相关参数
        struct 
        {
            MCAstar::PathSimplificationType PATH_SIMPLIFICATION_TYPE = MCAstar::PathSimplificationType::DouglasPeucker; // 去除冗余点方法，共有三种
            double THRESHOLD = 1.0;     // 去除冗余点的阈值。DouglasPeucker 和 DistanceThreshold 方法单位为m， AngleThreshold 方法单位为rad
        } path_simplification_params;

        // 贝塞尔曲线平滑相关参数
        struct
        {
            double T_STEP = 0.01;       // 贝塞尔曲线的步长，t∈(0, 1)， t越大，生成的贝塞尔曲线离散点的密度就越大。
        } bezier_curve_params;

        // 降采样相关参数
        struct
        {
            double INTERVAL = 0.3;      // 根据res_转化到实际地图上后的尺寸进行判断，降采样后的路径上两点间距至少大于INTERVAL
        } downsampling_params;
    };

private:
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
        uint8_t cost = 0;   // 栅格中的代价值
        double g = 0;       // 起点到该点已探索的可通行距离代价值
        double h = 0;       // 该点到终点的启发值
        double w = 1;       // 该点的权重值，为动态加权
        double w_cost = 0;  // 该点到终点矩形区域内代价值的和，为计算权重的中间量
        double f = 0;       // 该点总共的代价值 f = g + w * h
        NodeType type = NodeType::UNKNOWN;  // 节点种类，标识是否已探索
        Node * parent_node = nullptr;       // 该节点的父节点
        Direction direction_to_parent = Direction::UNKNOWN;     // 该节点相对父节点的方位


        /// @brief 节点比较重载，用于优先队列
        bool operator<(const Node & other) const { return f > other.f; }
        struct NodePointerCmp
        {
            bool operator()(const Node * const l, const Node * const r) const { return l->f > r->f; }
        };
    };
    
public:
    MCAstar() = default;
    ~MCAstar() = default;

    /// @brief 对规划器相关变量进行初始化设置，进行参数拷贝设置
    /// @param params 传入的参数
    void initParams(const GlobalPlannerParams & params) override;
    /// @brief 对输入的map的进行预处理膨胀、过滤小代价值栅格，然后设置成为规划器中需要使用的地图
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
    /// @brief 获得处理后的地图，即算法内部真正使用的，经过膨胀、忽视小代价值后的地图
    /// @param map 地图将存入该变量
    /// @return 存入是否成功
    bool getProcessedMap(cv::Mat & map) const override;
    /// @brief 通过给定的地图、起点、终点规划出一条从起点到终点的路径。
    /// @param path 规划出的路径。该路径是栅格坐标系下原始路径点，没有冗余点剔除、平滑、降采样操作。
    /// @return 是否规划成功
    bool getRawPath(std::vector<cv::Point2i> & path) override;
    /// @brief 通过给定的地图、地图信息、起点、终点规划出一条从起点到终点的平滑路径。
    /// @param path 规划出的路径。该路径是真实地图下的坐标点，进行冗余点剔除、分段三阶贝塞尔曲线平滑、降采样操作。并且补齐0.5个单位长度的栅格偏差。
    /// @return 是否规划成功
    bool getSmoothPath(std::vector<cv::Point2d> & path) override;
    /// @brief 打印所有的信息，包括规划器参数信息、规划地图信息、规划结果信息，并可以将结果保存到指定路径中。调用内部辅助helper实现
    /// @param save 是否保存到本地
    /// @param save_dir_path 保存的路径
    void showAllInfo(const bool save, const std::string & save_dir_path) const override { /*helper_.showAllInfo(save, save_dir_path);*/ }

private:
    MCAstarParams params_;                  // 规划器参数
    
    std::vector<std::vector<Node>> map_;    // 实际使用地图
    Node * start_node_ = nullptr;           // 起点节点
    Node * end_node_   = nullptr;           // 终点节点

    /// @brief 根据启发类型，得到节点n到终点的启发值
    /// @param n 待计算的节点，计算该节点到终点的启发值
    void getH(Node * const n) const;
    /// @brief 计算p点到终点的启发权重
    /// @param p 待计算的点，计算该点到终点的启发权重
    void getW(Node * const n) const;
    /// @brief 通过代价地图计算得到的原始路径，未经过去除冗余点、平滑操作
    /// @param raw_nodes 输出规划的原始结果
    /// @return 规划是否成功。如果起点和终点不可达则规划失败
    bool generateRawNodes(std::vector<Node *> & raw_nodes);
    /// @brief 将节点Node数据类型转换成便于后续计算的Point数据类型
    /// @param nodes 输入的待转换节点
    /// @param path 输出的路径
    void nodesToPath(const std::vector<Node *> & nodes, std::vector<cv::Point2i> & path) const;
    /// @brief 根据路径节点之间的关系，去除中间的冗余点，只保留关键点，如：起始点、转弯点
    /// @param raw_path 输入的待去除冗余点的路径
    /// @param reduced_path 输出的去除冗余点后的路径
    void removeRedundantPoints(const std::vector<cv::Point2i> & raw_path, std::vector<cv::Point2i> & reduced_path) const;
    /// @brief 使用分段三阶贝塞尔曲线进行平滑。该函数将栅格整数坐标xy的值进行平滑，得到平滑路径后再消除栅格偏差的0.5个单位长度
    /// @param raw_path 待平滑路径
    /// @param soomth_path 输出平滑后的路径
    /// @param control_nums_per_subpath 每个子路径的点的数目
    /// @return 平滑操作是否成功。若缺少地图信息或输入路径为空，返回false
    bool smoothPath(const std::vector<cv::Point2i> & reduced_path, std::vector<cv::Point2d> & smooth_path) const;
    /// @brief 对路径进行降采样。在进行完贝塞尔曲线平滑后进行，避免规划出的路径过于稠密
    /// @param path 待降采样的路径
    /// @param dis 两点间最小间距
    void downsampling(std::vector<cv::Point2d> & path) const;
    /// @brief 将当前地图中的参数全部初始化。一般在完成一次规划的所有步骤后进行。
    void resetMap();
};
