#pragma once
#include <random>

#include "global_planning/planners/global_planner_interface.h"
#include "global_planning/tools/print_struct_and_enum.h"
#include "global_planning/tools/math.h"


/// @brief 基于RRT*(Rapidly-exploring Random Tree Star，启发快速探索随机树)算法的全局路径规划器。
/// 随机点生成和后续使用都是离散点而非栅格点，然后放在栅格地图中进行障碍物判断。
class RRTstar : public GlobalPlannerInterface
{
public:
    /// @brief 用于RRTstar规划器使用的参数类型
    struct RRTstarParams : public GlobalPlannerParams
    {
        ~RRTstarParams() = default;

        // 地图相关参数，uint16_t类型原本为uint8_t类型，但cout输出uint8_t类型时按照ASCII码输出字符，而非整数。
        struct
        {
            uint16_t OBSTACLE_THRESHOLD = 50;   // 地图中栅格代价值大于等于(>=)该值的栅格，会被视为障碍物，搜索过程中将直接跳过该栅格

            REGISTER_STRUCT(REGISTER_MEMBER(OBSTACLE_THRESHOLD));
        } map;

        struct
        {
            size_t ITERATOR_TIMES = 100000;     // 最大迭代次数
            double GOAL_SAMPLE_RATE = 0.1;      // 终点采样率，即每走一步，有多少的概率随机选取终点作为新的终点
            double GOAL_DIS_TOLERANCE = 2.0;    // 新增长节点若与终点的距离在该范围内，则直接将终点作为新增长节点
            double STEP_SIZE = 3.0;             // 每次扩展节点的步长
            double NEAR_DIS = 10.0;             // 在扩展完新节点后，再该范围内重新璇姐父节点、重新排线

            REGISTER_STRUCT(REGISTER_MEMBER(ITERATOR_TIMES),
                            REGISTER_MEMBER(GOAL_SAMPLE_RATE),
                            REGISTER_MEMBER(GOAL_DIS_TOLERANCE),
                            REGISTER_MEMBER(STEP_SIZE),
                            REGISTER_MEMBER(NEAR_DIS));
        } sample;

        REGISTER_STRUCT(REGISTER_MEMBER(map),
                        REGISTER_MEMBER(sample));
    };

    /// @brief 用于RRTstar规划器的辅助类，实现数据记录和结果打印
    class RRTstarHelper : public GlobalPlannerHelper
    {
    public:
        using GlobalPlannerHelper::GlobalPlannerHelper;
        ~RRTstarHelper() = default;

        struct
        {
            size_t node_nums = 0;           // 搜索到的有效节点个数，即最终所有树的节点个数。
            size_t node_counter = 0;        // 搜索过程中所有创建的节点个数，包含无效节点，也可认为是程序的迭代次数
            size_t path_length = 0;         // 规划出的路径长度，单位为栅格数，包含起点和终点
            double cost_time = 0;           // 搜索总耗时，单位ms

            std::vector<cv::Point2d> cur_points;    // 当前节点
            std::vector<cv::Point2d> par_points;    // 当前节点父节点

            REGISTER_STRUCT(REGISTER_MEMBER(node_nums),
                            REGISTER_MEMBER(node_counter),
                            REGISTER_MEMBER(path_length),
                            REGISTER_MEMBER(cost_time));
        } sample;

    public:
        /// @brief 清空当前记录的所有结果信息，便于下次记录
        void resetResultInfo() override
        {
            sample.node_nums = 0;
            sample.node_counter = 0;
            sample.path_length = 0;
            sample.cost_time = 0;
            sample.cur_points.clear();
            sample.par_points.clear();
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
    struct TreeNode
    {
        TreeNode() : pos(0.0, 0.0), cost(0.0), parent(nullptr) {}
        TreeNode(const cv::Point2d & pos_, double cost_, TreeNode * const parent_) : pos(pos_), cost(cost_), parent(parent_) {}
        TreeNode(const TreeNode & other) = delete;
        TreeNode(TreeNode && other) = delete;
        TreeNode & operator=(const TreeNode & other) = delete;
        TreeNode & operator=(TreeNode && other) = delete;
        ~TreeNode() = default;

        cv::Point2d pos;    // 当前节点位置，是在栅格坐标系下的离散值
        double cost = 0.0;  // 当前节点到根节点的代价值。目前为距离代价值
        TreeNode * parent;  // 当前节点的父节点
    };

public:
    RRTstar() : helper_(this) { planner_name_ = "RRTstar"; }
    RRTstar(const RRTstar & other) = delete;
    RRTstar(RRTstar && other) = delete;
    RRTstar & operator=(const RRTstar & other) = delete;
    RRTstar & operator=(RRTstar && other) = delete;
    ~RRTstar() = default;

    /// @brief 对规划器相关变量进行初始化设置，进行参数拷贝设置
    /// @param params 传入的参数
    void initParams(const GlobalPlannerParams & params) override;
    /// @brief 对输入的map的进行二值化处理，然后设置成为规划器中需要使用的地图
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
    /// @param auxiliary_info 辅助信息。该信息是原始地图坐标系下路径规划过程中的各种关键路径点信息。此处包括RRT扩展到的其他节点。
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
    RRTstarParams params_;                  // 规划器参数
    RRTstarHelper helper_;                  // 规划器辅助

    cv::Mat map_;                           // 代价地图
    cv::Point2d start_point_;               // 起点，真实地图上的起点落在栅格地图中的离散坐标
    cv::Point2d end_point_;                 // 终点，真实地图上的终点落在栅格地图中的离散坐标

    std::vector<TreeNode *> tree_list_;     // 当前所有节点 

    /// @brief 寻找距离当前随机节点最近的现有节点的下标
    /// @param rd_node 当前随机节点
    /// @return 现有节点中距离随机节点最近节点nearest_node的下标
    size_t nearest_node_index(const cv::Point2d & rd_pt) const;
    /// @brief 判断当前点是否位于地图内部
    /// @param pt 待判断的点
    /// @return true —— 在内部； false —— 不在内部
    bool is_inside_map(const cv::Point2d & pt) const;
    /// @brief 地图上检测pt1和pt2连线上是否存在障碍物节点。用于判断新点new_point和最近点nearest_point之间是否存在障碍物。
    /// @param pt1 第一个点
    /// @param pt2 第二个点
    /// @return 是否发生碰撞。true —— 碰撞； false —— 不碰撞
    bool check_collision(const cv::Point2d & pt1, const cv::Point2d & pt2) const;
    /// @brief 查找当前新节点new_pt周围选定范围内的、与新节点连线不碰撞的所有节点下标，并计算到其的距离代价值
    /// @param new_pt 待查找周围成员的新节点
    /// @return 所有附近成员的数组，其中每个成员是pair<size_t, double>，分别保存索引和距离值
    std::vector<std::pair<size_t, double>> near_nodes_info(cv::Point2d & new_pt) const;
    /// @brief 计算当前搜索范围节点中代价值最低的节点，确定新节点对应的父节点
    /// @param info 周围节点的信息
    /// @return 确定的父节点下标
    size_t choose_parent(const std::vector<std::pair<size_t, double>> & info) const;
    /// @brief 对当前选定范围内的节点进行重新排线。若周围节点near_node到起点的代价值大于途径new_node到起点的代价值，则near_node父节点更改为new_node
    /// @param new_node 当前选定的new_node
    /// @param info new_node周围节点的信息
    void rewire(TreeNode * const new_node, const std::vector<std::pair<size_t, double>> & info) const;
};