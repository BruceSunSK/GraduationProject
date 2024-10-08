#pragma once
#include <queue>
#include <algorithm>

#include "global_planning/global_planner_interface.h"
#include "global_planning/tools/print_struct_and_enum.h"


/// @brief 使用Astar算法的规划结果，无任何改进
class Astar : public GlobalPlannerInterface
{
public:
    enum class HeuristicsType : uint8_t
    {
        None, 
        Manhattan, 
        Euclidean,
        Chebyshev,
        Octile
    };

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
        void showAllInfo(const bool save = false, const std::string & save_dir_path = "") override;
        /// @brief 清空当前记录的所有结果信息，便于下次记录
        void resetResultInfo() override
        {
            search_result.node_nums = 0;
            search_result.node_counter = 0;
            search_result.cost_time = 0;
        }

    private:
        std::string paramsInfo() override;
        std::string mapInfo() override;
        std::string resultInfo() override;
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

        cv::Point2i point;
        double cost = 0;

        double g = 0;
        double h = 0;
        double f = 0;
        NodeType type = NodeType::UNKNOWN;
        Node * parent_node = nullptr;

        // 用于节点比较
        bool operator<(const Node & other) const { return f > other.f; }
        class NodePointerCmp
        {
        public:
            bool operator()(Node * l, Node * r) const { return l->f > r->f; }
        };
    };

public:
    Astar() : helper_(this) {}
    ~Astar() = default;

    void initParams(const GlobalPlannerParams & params) override;
    bool setMap(const cv::Mat & map) override;
    bool setStartPoint(const int x, const int y) override;
    bool setStartPoint(const cv::Point2i p) override;
    bool setEndPoint(const int x, const int y) override;
    bool setEndPoint(const cv::Point2i p) override;

    bool getProcessedMap(cv::Mat & map) override;
    bool getRawPath(std::vector<cv::Point2i> & path) override;
    bool getSmoothPath(std::vector<cv::Point2d> & path) override;
    void showAllInfo(const bool save, const std::string & save_dir_path) override { helper_.showAllInfo(save, save_dir_path); }

private:
    AstarParams params_;
    AstarHelper helper_;

    std::vector<std::vector<Node>> map_;
    Node * start_node_ = nullptr;
    Node * end_node_   = nullptr;

    double getH(const cv::Point2i & p);
};

using HeuristicsType = Astar::HeuristicsType;
REGISTER_ENUM(HeuristicsType);
