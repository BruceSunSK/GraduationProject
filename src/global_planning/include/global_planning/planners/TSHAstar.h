#pragma once
#include <queue>
#include <unordered_map>
#include <algorithm>

#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>

#include "global_planning/planners/global_planner_interface.h"
#include "global_planning/tools/print_struct_and_enum.h"
#include "global_planning/tools/math.h"
#include "global_planning/curve/bezier_curve.h"
#include "global_planning/curve/bspline_curve.h"
#include "global_planning/curve/cubic_spline_curve.h"
#include "global_planning/path/simplification.h"
#include "global_planning/path/reference_path.h"
#include "global_planning/smoothers/discrete_point_smoother.h"


/// @brief Two-Stage Hybird Astar (TSHAstar)
/// @details 总共可以分为预处理、搜索、采样三个步骤
/// 【预处理】：主要是实现代价地图离散化的效果。此处本身使用的代价地图就是离散的，但是还是进行一步预处理，实现膨胀、距离地图的生成。
/// [1] 实现栅格代价值离散化，0~100和255。代价值低处更易通过；OBSTACLE值(默认为100)以上为障碍物，彻底不可通过；255为未探索区域（与建图相关），直接当做障碍物处理。
///     具体操作为：使用预处理对原始离散地图进行膨胀。只对代价值处于[min, max]区间的栅格进行按照距离反比进行膨胀，且乘以系数K，目前为1.3
///     膨胀后的地图若代价值小于COST_THRESHOLD，则置为0，降低较小值带来的无效计算。
///
/// 【第一大步】：使用改进的Astar在栅格地图上进行搜索，建立一条起点到终点的初步路径，然后对这条路径进行平滑，作为参考路径。
///             这一步主要的作用是，利用搜索方法的快速性，压缩解空间，加快后续采样步骤的速度。
/// [2] 搜索领域从8领域扩展改为5领域扩展。原因是8领域扩展会导致搜索步长过大，出现局部最优，且不易处理障碍物密集的区域。
///     1. 4领域扩展：左、右、上、下
///        效果非常差。原始路线存着硬弯，再经过冗余点去除和平滑后效果更差。
///     2. 5领域扩展：左、右、上、下、对角线
///        效果最好。实际扩展到的节点数目和8领域基本一致，意味着路径和8领域是一样最优的；但是5领域可以减少扩展节点的次数，降低计算量。
///     3. 8领域扩展：左、右、上、下、左上、右上、左下、右下
///        效果一般。虽然可以实现最优路径，但与搜索方向相反的节点基本无需搜索，导致不必要的计算。    
/// [3] 代价函数 f = g + w * h
///      1. h用法和原版AStar保持一致，为距离启发代价。此处选择Euclidean距离启发。
///         None：无启发类型，此时退化会成为Dijkstra算法。
///         Manhattan：曼哈顿距离，计算公式为 |x1 - x2| + |y1 - y2|，适用于对角线距离较远的情况。
///         Euclidean：欧式距离，计算公式为 sqrt((x1 - x2)^2 + (y1 - y2)^2)，适用于直线距离较远的情况。
///         Chebyshev：切比雪夫距离，计算公式为 max(|x1 - x2|, |y1 - y2|)，适用于对角线距离较近的情况。
///         Octile：对角线距离，计算公式为 max(dx, dy) + (sqrt(2) - 1) * min(dx, dy)
///      2. g为综合距离代价值。考虑距离时乘以可通行度代价系数和转向代价系数的总已探索代价。
///         2.1 具体实现为：根据栅格代价值计算出一个大于1的可通行度代价系数与原有的g相乘；根据栅格转向角度，计算不同转向代价系数与g相乘。
///         2.2 cost为栅格的可通行代价；C为转向代价系数；g0为距离代价，等价于原版astar中的g
///             g = \sigma gi = \sigma exp(k * costi / 100) * Ci * g0i
///             其中，k为自定义系数，k值越大，则远离障碍的趋势越明显，但也会导致出现绕弯。目前k = 2
///                  根据栅格转向角度(同向直行，同向斜行，垂向直行，反向斜行)，赋予不同转向代价系数C为：c1, c2, c3, c4，目前分别为1.0, 1.4, 2.0, 3.0。
///         2.3 使用可通行度代价系数可以有效利用离散的栅格代价值，而非二值化；
///             使用转向代价系数可以针对不能倒车的情形进行约束，且能够通过调参实现不同的倾向，而非5领域扩展法的一刀切。
///      3. 【效果很差，暂未使用】w为加权系数，此处选择为动态加权。
///         【无论是下述当前点到终点间的矩形区域，还是当前点周围的部分区域，效果都很差】
///         引入障碍物密度的概念P(p1, p2) = (\sigma cost) / (100 * (|p1.x - p2.x| + 1) * (|p1.y - p2.y| + 1)), \sigma cost为矩形区域内的总代价值。
///         然后，w = (1 - lnP), P ∈ (0, 1), w ∈ (1, ∞)。
///         可以实现在障碍物密集的区域实现避免搜索步长过大，出现局部最优，有效避开障碍物；在障碍物较少的区域中，加快搜索，减少搜索栅格个数。
/// [4] 去除冗余点
///      1. DouglasPeucker法。以起点终点连线为基线，找到最远的点，超过threshold则只保留起点终点；否则以该点递归左右两侧。
///      2. DistanceThreshold法。以起点开始依次向该点之后连线，判断该点之后的点到该直线距离，超过阈值则保留该点，并重置上述过程；否则舍去，继续向后。
///      3. AngleThreshold法。逻辑和DistanceThreshold法一致，只不过判断方法变成了角度（<新点，原直线起点连线> 和 <原直线>的角度）。
///      4. DPPlus法。DPPlus法是改进的DouglasPeucker法。
///         4.1 初始过程和DouglasPeucker法一致，以起点终点连线为基线，对于每一个点，计算其与基线的距离，得到最远具体的点。
///         4.2 如果最远距离超过DISTANCE_THRESHOLD，则直接对左右两侧进行递归处理；否则，进行如下障碍物判断：
///         4.3 对该起点和终点使用Bresenham算法，得到该起点终点连线(所连直线宽度由LINE_THRESHOLD控制)上的所有点位置，判断其所在栅格代价值是否大于等于OBSTACLE_THRESHOLD。
///             若存在任意一个栅格是障碍物，则认为该两点直接存在障碍物，无法直接连接，不进行简化。仍选择最原距离点两侧分别进行递归处理。
///             若所有连接点栅格均不存在障碍物，则认为该两点间无障碍物，可以连接，进行简化，只保留起点和终点。
/// [5] 使用曲线平滑路径。
///     5.1 引入贝塞尔曲线进行分段平滑。
///         1. 首先将路径点去除起点和终点后每两个划分为1组。第一组额外包括起点，最后一组额外包括终点
///         2. 然后第i组的第二个点与第i+1组的第一个点线性插值的中点作为新插入节点，同时作为第i组和第i+1组的成员。
///         3. 这样每组都拥有四个节点成员，使用三阶贝塞尔曲线进行平滑。这样在分段处基本保持连续和曲率平滑。
///         4. 最后一组可能由于点的数量不够，只有三个点，此时使用二阶贝塞尔曲线进行平滑。
///     5.2 引入B样条曲线进行全局平滑。对去除冗余点后的路径直接使用4阶3次B样条曲线进行全局平滑。
///     总体使用而言，B样条更快，更平滑。
/// [6] 由于生成的贝塞尔曲线是稠密的，因此手动进行降采样。按照两点间距离进行判断，使得路径离散。
///
/// 【第二大步】：将第一大步的参考路径转换到Frenet坐标系下，在参考路径两侧进行撒点采样，然后使用DP快速确定满足车辆运动学的解空间，得到上下边界约束。
///             然后利用确定好的上下边界约束，再次对参考路径进行平滑，此时要考虑运动学约束，得到最终的全局路径。
class TSHAstar : public GlobalPlannerInterface
{
public:
    /// @brief 领域扩展类型
    enum class NeighborType : uint8_t
    {
        FourConnected,
        FiveConnected,
        EightConnected
    };
    
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
        AngleThreshold,
        DPPlus
    };

    /// @brief 曲线平滑方法类型
    enum class PathSmoothType : uint8_t
    {
        Bezier,
        BSpline
    };

    /// @brief 用于TSHAstar规划器使用的参数类型
    struct TSHAstarParams : public GlobalPlannerParams
    {
        ~TSHAstarParams() = default;

        // 地图相关参数，对应第一大步，uint16_t类型原本为uint8_t类型，但cout输出uint8_t类型时按照ASCII码输出字符，而非整数。
        struct 
        {
            double EXPANDED_K = 1.3;                // 地图膨胀时，膨胀系数
            uint16_t EXPANDED_MIN_THRESHOLD = 0;    // 原始地图栅格代价值大于等于(>=)该值的栅格，才进行膨胀
            uint16_t EXPANDED_MAX_THRESHOLD = 100;  // 原始地图栅格代价值小于等于(<=)该值的栅格，才进行膨胀
            uint16_t COST_THRESHOLD = 10;           // 膨胀地图中栅格代价值大于等于(>=)该值的栅格，会被视为有代价值的栅格，否则代价值为0
            uint16_t OBSTACLE_THRESHOLD = 100;      // 膨胀地图中栅格代价值大于等于(>=)该值的栅格，会被视为障碍物，搜索过程中将直接跳过该栅格

            REGISTER_STRUCT(REGISTER_MEMBER(EXPANDED_K),
                            REGISTER_MEMBER(EXPANDED_MIN_THRESHOLD),
                            REGISTER_MEMBER(EXPANDED_MAX_THRESHOLD),
                            REGISTER_MEMBER(COST_THRESHOLD),
                            REGISTER_MEMBER(OBSTACLE_THRESHOLD))
        } map;

        // 搜索过程相关参数，对应第二大步。
        struct
        {
            // 代价函数相关参数
            struct
            {
                TSHAstar::NeighborType NEIGHBOR_TYPE = TSHAstar::NeighborType::FiveConnected;     // 领域扩展类型，共有三种
                TSHAstar::HeuristicsType HEURISTICS_TYPE = TSHAstar::HeuristicsType::Euclidean;   // 启发值类型，共有五种
                double TRAV_COST_K = 2.0;                   // 计算可通行度代价值时的系数
                double TURN_COST_STRAIGHT = 1.0;            // 同向直行转向代价系数，C1
                double TURN_COST_SLANT = 1.4;               // 同向斜行转向代价系数，C2
                double TURN_COST_VERTICAL = 2.0;            // 垂向直行转向代价系数，C3
                double TURN_COST_REVERSE_SLANT = 3.0;       // 反向斜行转向代价系数，C4

                REGISTER_STRUCT(REGISTER_MEMBER(NEIGHBOR_TYPE),
                                REGISTER_MEMBER(HEURISTICS_TYPE),
                                REGISTER_MEMBER(TRAV_COST_K),
                                REGISTER_MEMBER(TURN_COST_STRAIGHT),
                                REGISTER_MEMBER(TURN_COST_SLANT),
                                REGISTER_MEMBER(TURN_COST_VERTICAL),
                                REGISTER_MEMBER(TURN_COST_REVERSE_SLANT))
            } path_search;

            // 冗余点去除相关参数
            struct
            {
                TSHAstar::PathSimplificationType PATH_SIMPLIFICATION_TYPE = TSHAstar::PathSimplificationType::DPPlus; // 去除冗余点方法，共有四种
                double DISTANCE_THRESHOLD = 1.5;    // 去除冗余点时的距离阈值，仅在DouglasPeucker、DistanceThreshold和DPPlus方法中使用。单位为m。
                double ANGLE_THRESHOLD = 0.17;      // 去除冗余点时的角度阈值，仅在AngleThreshold方法中使用。单位为rad。
                uint16_t OBSTACLE_THRESHOLD = 70;   // 去除冗余点时的障碍物阈值，仅在DPPlus方法中使用。范围为[0, 100]。在去除冗余点连线上，如果有大于等于≥该值的栅格，将被视为存在障碍物，取消本次连线。建议略小于map_params.OBSTACLE_THRESHOLD。
                double LINE_WIDTH = 1.0;            // 去除冗余点时的线宽，仅在DPPlus方法中使用。单位为m。在去除冗余点连线上，按照该宽度进行障碍物检测。
                double MAX_INTAVAL = 8.0;           // 去除冗余点时上采样的最大间隔，仅在DPPlus方法中使用。单位为m。

                REGISTER_STRUCT(REGISTER_MEMBER(PATH_SIMPLIFICATION_TYPE),
                                REGISTER_MEMBER(DISTANCE_THRESHOLD),
                                REGISTER_MEMBER(ANGLE_THRESHOLD),
                                REGISTER_MEMBER(OBSTACLE_THRESHOLD),
                                REGISTER_MEMBER(LINE_WIDTH),
                                REGISTER_MEMBER(MAX_INTAVAL))
            } path_simplification;

            // 曲线平滑相关参数
            struct
            {
                TSHAstar::PathSmoothType PATH_SMOOTH_TYPE = TSHAstar::PathSmoothType::BSpline;    // 曲线平滑方法，共有两种
                double T_STEP = 0.0005;     // 平滑曲线的步长，t∈(0, 1)， t越大，生成的曲线离散点的密度就越大。贝塞尔曲线是分段的，T_STEP作用于每一段；B样条曲线是全局的，T_STEP作用于全局。

                REGISTER_STRUCT(REGISTER_MEMBER(PATH_SMOOTH_TYPE),
                                REGISTER_MEMBER(T_STEP))
            } path_smooth;

            // 数值优化平滑相关参数
            struct
            {
                double S_INTERVAL = 5.0;            // 曲线平滑后的路径进行均匀采样的间隔，单位为m。
                double WEIGTH_SMOOTH = 100.0;       // 权重平滑系数，用于优化曲线时描述路径点折弯程度。
                double WEIGTH_LENGTH = 1.0;         // 长度平滑系数，用于优化曲线时描述路径点总长度。
                double WEIGTH_DEVIATION = 10.0;     // 偏差平滑系数，用于优化曲线时描述优化后路径点与原始路径点的偏差程度。

                REGISTER_STRUCT(REGISTER_MEMBER(S_INTERVAL),
                                REGISTER_MEMBER(WEIGTH_SMOOTH),
                                REGISTER_MEMBER(WEIGTH_LENGTH),
                                REGISTER_MEMBER(WEIGTH_DEVIATION))
            } path_optimization;

            REGISTER_STRUCT(REGISTER_MEMBER(path_search),
                            REGISTER_MEMBER(path_simplification),
                            REGISTER_MEMBER(path_smooth),
                            REGISTER_MEMBER(path_optimization))
        } search;

        // 采样过程相关参数，对应第三大步。
        struct 
        {
        } sample;
        
        REGISTER_STRUCT(REGISTER_MEMBER(map),
                        REGISTER_MEMBER(search));
    };

    /// @brief 用于TSHAstar规划器使用的辅助类，实现数据记录和结果打印
    class TSHAstarHelper : public GlobalPlannerHelper
    {
    public:
        using GlobalPlannerHelper::GlobalPlannerHelper;
        ~TSHAstarHelper() = default;

        struct 
        {
            struct
            {
                size_t raw_node_nums = 0;       // 搜索到节点的数量，不包括障碍物节点
                size_t raw_node_counter = 0;    // 搜索到节点的次数，会重复计算同一个节点，不包括障碍物节点
                size_t raw_path_size = 0;       // 规划出的路径点个数，单位为栅格数，包含起点和终点
                double cost_time = 0.0;         // 搜索总耗时，单位ms

                std::vector<cv::Point2d> nodes;     // 按顺序搜索到的节点，不包括障碍物节点
                std::vector<cv::Point2d> raw_path;  // 原始的规划路径结果

                REGISTER_STRUCT(REGISTER_MEMBER(raw_node_nums),
                                REGISTER_MEMBER(raw_node_counter),
                                REGISTER_MEMBER(raw_path_size),
                                REGISTER_MEMBER(cost_time))
            } path_search;

            struct
            {
                size_t reduced_path_size = 0;           // 去除冗余点后的路径点个数
                double cost_time = 0.0;                 // 去除冗余点后搜索总耗时，单位ms

                std::vector<cv::Point2d> reduced_path;  // 去除冗余点后的规划路径结果

                REGISTER_STRUCT(REGISTER_MEMBER(reduced_path_size),
                                REGISTER_MEMBER(cost_time));
            } path_simplification;

            struct
            {
                size_t smooth_path_size = 0;            // 曲线平滑后的路径点个数
                double cost_time = 0.0;                 // 曲线平滑后搜索总耗时，单位ms

                std::vector<cv::Point2d> smooth_path;   // 曲线平滑后的路径结果

                REGISTER_STRUCT(REGISTER_MEMBER(smooth_path_size),
                                REGISTER_MEMBER(cost_time))
            } path_smooth;

            struct
            {
                size_t sample_path_size = 0;            // 采样后路径的大小
                double sample_path_length = 0.0;        // 采样后路径的长度
                size_t optimized_path_size = 0;         // 优化后路径的大小
                double optimized_path_length = 0.0;     // 优化后路径的长度
                double cost_time = 0.0;                 // 整个优化过程总耗时，单位ms

                std::vector<cv::Point2d> sample_path;       // 使用三次样条插值降采样后得到的路径
                std::vector<cv::Point2d> optimized_path;    // 优化后的路径
                
                REGISTER_STRUCT(REGISTER_MEMBER(sample_path_size),
                                REGISTER_MEMBER(sample_path_length),
                                REGISTER_MEMBER(optimized_path_size),
                                REGISTER_MEMBER(optimized_path_length),
                                REGISTER_MEMBER(cost_time))
            } path_optimization;
            
            REGISTER_STRUCT(REGISTER_MEMBER(path_search),
                            REGISTER_MEMBER(path_simplification),
                            REGISTER_MEMBER(path_smooth),
                            REGISTER_MEMBER(path_optimization))
        } search;

    public:
        /// @brief 清空当前记录的所有结果信息，便于下次记录
        void resetResultInfo() override
        {
            search.path_search.raw_node_nums = 0;
            search.path_search.raw_node_counter = 0;
            search.path_search.raw_path_size = 0;
            search.path_search.cost_time = 0.0;
            search.path_search.nodes.clear();
            search.path_search.raw_path.clear();
            
            search.path_simplification.reduced_path_size = 0;
            search.path_simplification.cost_time = 0.0;
            search.path_simplification.reduced_path.clear();

            search.path_smooth.smooth_path_size = 0;
            search.path_smooth.cost_time = 0.0;
            search.path_smooth.smooth_path.clear();

            search.path_optimization.sample_path_size = 0;
            search.path_optimization.sample_path_length = 0.0;
            search.path_optimization.optimized_path_size = 0;
            search.path_optimization.optimized_path_length = 0.0;
            search.path_optimization.cost_time = 0.0;
            search.path_optimization.sample_path.clear();
            search.path_optimization.optimized_path.clear();
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
    /// @brief 用于描述搜索过程中的单个栅格节点
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
        /// @brief 包含最多八个方向的相邻节点的偏移量表，根据需求调用不同index的值，使用OpenCV坐标系
        static const int neighbor_offset_table[8][2];
        /// @brief 不用方向枚举的映射标，输入索引，输出方向
        static const Direction direction_table[8];

        /// @brief 节点属性
        cv::Point2i point;  // 栅格的xy值
        uint8_t cost = 0;   // 栅格中的代价值
        double g = 0;       // 起点到该点已探索的可通行距离代价值
        double h = 0;       // 该点到终点的启发值
        double w = 1;       // 该点的权重值，为动态加权。
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
    TSHAstar() : helper_(this) { planner_name_ = "TSHAstar"; }
    TSHAstar(const TSHAstar & other) = delete;
    TSHAstar(TSHAstar && other) = delete;
    TSHAstar & operator=(const TSHAstar & other) = delete;
    TSHAstar & operator=(TSHAstar && other) = delete;
    ~TSHAstar() = default;

    /// @brief 对规划器相关变量进行初始化设置，进行参数拷贝设置
    /// @param params 传入的参数
    void initParams(const GlobalPlannerParams & params) override;
    /// @brief 对输入的map的进行预处理膨胀、过滤小代价值栅格，然后设置成为规划器中需要使用的地图
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
    /// @brief 设置规划路径的起点的朝向。
    /// @param yaw x轴为0，右手坐标系；单位为弧度；范围为[-pi, pi]。
    /// @return 该设置有效。规划器无设置起点朝向的功能时，返回false，否则返回true。
    bool setStartPointYaw(const double yaw) override;
    /// @brief 设置规划路径的终点。以真实地图坐标形式，而非行列形式。
    /// @param x 真实地图坐标系的x值
    /// @param y 真实地图坐标系的y值
    /// @return 该点是否能够成为终点。即该点在地图内部且不在障碍物上。
    bool setEndPoint(const double x, const double y) override;
    /// @brief 设置规划路径的终点。以真实地图坐标形式，而非行列形式。
    /// @param p 真实地图坐标系的点
    /// @return 该点是否能够成为终点。即该点在地图内部且不在障碍物上。
    bool setEndPoint(const cv::Point2d & p) override;
    /// @brief 设置规划路径的终点的朝向。
    /// @param yaw x轴为0，右手坐标系；单位为弧度；范围为[-pi, pi]。
    /// @return 该设置有效。规划器无设置终点朝向的功能时，返回false，否则返回true。
    bool setEndPointYaw(const double yaw) override;
    /// @brief 获得处理后的地图，即算法内部真正使用的，经过膨胀、忽视小代价值后的地图
    /// @param map 地图将存入该变量
    /// @return 存入是否成功
    bool getProcessedMap(cv::Mat & map) const override;
    /// @brief 通过给定的地图、起点、终点规划出一条从起点到终点的最终路径。
    /// @param path 规划出的路径。该路径是原始地图坐标系下原始路径点。
    /// @param auxiliary_info 辅助信息。该信息是原始地图坐标系下路径规划过程中的各种关键路径点信息。此处包括TSHAstar按顺序扩展到的节点、原始路径点、剔除冗余点后的路径点。
    /// @return 是否规划成功
    bool getPath(std::vector<cv::Point2d> & path, std::vector<std::vector<cv::Point2d>> & auxiliary_info) override;
    /// @brief 打印所有的信息，包括规划器参数信息、规划地图信息、规划结果信息，并可以将结果保存到指定路径中。调用内部辅助helper实现
    /// @param save 是否保存到本地
    /// @param save_dir_path 保存的路径
    void showAllInfo(const bool save = false, const std::string & save_dir_path = "") const override { helper_.showAllInfo(save, save_dir_path); }

private:
    TSHAstarParams params_;         // 规划器参数
    TSHAstarHelper helper_;         // 规划器辅助

    // 通用
    cv::Mat discrete_map_;          // 离散代价值的障碍物地图，正常范围在[0, 100]，也包括255未探明的情况
    cv::Mat binary_map_;            // 代价值二值化后的障碍物地图，未探明情况视为障碍物。即[0, OBSTACLE_THRESHOLD)值为0，[OBSTACLE_THRESHOLD, 255]值为255
    cv::Mat distance_map_;          // 在二值化地图中进行distanceTransform变换的结果，用于描述每个栅格到障碍物的距离
    cv::Point2d start_point_;       // 起点，真实地图上的起点落在栅格地图中的离散坐标
    cv::Point2d end_point_;         // 终点，真实地图上的终点落在栅格地图中的离散坐标
    
    // 搜索过程
    std::vector<std::vector<Node>> search_map_;     // 在搜索规划中实际使用地图
    Node * start_node_ = nullptr;                   // 起点节点
    Node * end_node_ = nullptr;                     // 终点节点

    // 采样过程

    
    /// @brief 根据配置参数，确定当前当前节点的相邻节点在Node::neightbor_offset_table中的索引值列表
    /// @param n 当前节点
    /// @return Node::neightbor_offset_table中的索引值列表
    std::vector<size_t> getNeighborsIndex(const Node * const n) const;
    /// @brief 准确讲应为getGi，即返回节点n到其父节点的G综合代价值。即为带有可通行度代价和转向代价的综合距离代价值。
    /// @param n 待计算的节点，计算该节点到其父节点的综合距离代价值
    /// @param direction 待计算的节点相对父节点的方向
    /// @param par_direction 待计算的父节点相对其父节点的方向
    /// @return 综合距离代价值，用于进一步判断该该节点是否需要被更新。
    double getG(const Node * const n, const Node::Direction direction, const Node::Direction par_direction) const;
    /// @brief 根据启发类型，得到节点n到终点的启发值
    /// @param n 待计算的节点，计算该节点到终点的启发值
    void getH(Node * const n) const;
    /// @brief 计算p点到终点的启发权重。【效果很差，暂不使用】
    /// @param p 待计算的点，计算该点到终点的启发权重
    void getW(Node * const n) const;
    /// @brief 将当前地图中的参数全部初始化。一般在完成一次规划的所有步骤后进行。
    void resetSearchMap();
    /// @brief 通过代价地图计算得到的原始路径，未经过去除冗余点、平滑操作
    /// @param raw_nodes 输出规划的原始结果
    /// @return 规划是否成功。如果起点和终点不可达则规划失败
    bool generateRawNodes(std::vector<Node *> & raw_nodes);
    /// @brief 将节点Node数据类型转换成便于后续计算的Point数据类型，并将当前地图中的参数全部初始化
    /// @param nodes 输入的待转换节点
    /// @param path 输出的路径
    void nodesToPath(const std::vector<Node *> & nodes, std::vector<cv::Point2i> & path);
    /// @brief 根据路径节点之间的关系，去除中间的冗余点，只保留关键点，如：起始点、转向点
    /// @param raw_path 输入的待去除冗余点的路径
    /// @param reduced_path 输出的去除冗余点后的路径
    void removeRedundantPoints(const std::vector<cv::Point2i> & raw_path, std::vector<cv::Point2i> & reduced_path);
    /// @brief 使用分段三阶贝塞尔曲线/B样条进行平滑。该函数将栅格整数坐标xy的值进行平滑，得到平滑路径后再消除栅格偏差的0.5个单位长度
    /// @param raw_path 待平滑路径
    /// @param soomth_path 输出平滑后的路径
    /// @param control_nums_per_subpath 每个子路径的点的数目
    /// @return 平滑操作是否成功。若缺少地图信息或输入路径为空，返回false
    bool smoothPath(const std::vector<cv::Point2i> & reduced_path, std::vector<cv::Point2d> & smooth_path);
    /// @brief 将平滑后的路径利用三次样条曲线均匀降采样，并使用数据优化的方法进一步平滑，得到全局参考路径
    /// @param smooth_path 平滑路径
    /// @param reference_path 经过优化平滑的参考线路径
    /// @return 优化是否成功
    bool optimizePath(const std::vector<cv::Point2d> & smooth_path, Path::ReferencePath::Ptr & reference_path);
};

using NeighborType = TSHAstar::NeighborType;
REGISTER_ENUM(NeighborType);
using HeuristicsType = TSHAstar::HeuristicsType;
REGISTER_ENUM(HeuristicsType);
using PathSimplificationType = TSHAstar::PathSimplificationType;
REGISTER_ENUM(PathSimplificationType);
using PathSmoothType = TSHAstar::PathSmoothType;
REGISTER_ENUM(PathSmoothType);
