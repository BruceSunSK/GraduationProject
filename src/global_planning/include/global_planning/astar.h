#pragma once
#include <iostream>
#include <vector>
#include <queue>
#include <algorithm>

#include "global_planning/global_planner_interface.h"


class Astar : public GlobalPlannerInterface
{
public:
    enum class HeuristicsType
    {
        None, 
        Manhattan, 
        Euclidean,
        Chebyshev,
        Octile
    };

    struct AstarParams : public GlobalPlannerParams
    {
        ~AstarParams() {}

        // 地图相关参数
        struct 
        {
            uint8_t OBSTACLE_THRESHOLD = 50;       // 地图中栅格代价值大于等于(>=)该值的栅格，会被视为障碍物，搜索过程中将直接跳过该栅格
        } map_params;

        // 代价函数相关参数
        struct
        {
            Astar::HeuristicsType HEURISTICS_TYPE = Astar::HeuristicsType::Euclidean;   // 启发值类型，共有五种
        } cost_function_params;
    };

private:
    struct Node
    {
        enum class NodeType
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
    Astar();
    ~Astar();

    void initParams(const GlobalPlannerParams & params) override;
    bool setMap(const cv::Mat & map) override;
    bool setStartPoint(const int x, const int y) override;
    bool setStartPoint(const cv::Point2i p) override;
    bool setEndPoint(const int x, const int y) override;
    bool setEndPoint(const cv::Point2i p) override;

    bool getProcessedMap(cv::Mat & map) override;
    bool getRawPath(std::vector<cv::Point2i> & path) override;
    bool getSmoothPath(std::vector<cv::Point2d> & path) override;

private:
    AstarParams params_;

    std::vector<std::vector<Node>> map_;
    Node * start_node_;
    Node * end_node_;

    double getH(cv::Point2i p);
};
