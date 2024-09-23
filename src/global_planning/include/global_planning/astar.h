#pragma once
#include <iostream>
#include <vector>
#include <queue>
#include <algorithm>

#include "global_planning/global_planner_interface.h"

class Astar : public GlobalPlannerInterface
{
    enum class HeuristicsType
    {
        None, 
        Manhattan, 
        Euclidean,
        Chebyshev,
        Octile
    };

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
    Astar(HeuristicsType type = HeuristicsType::Euclidean);
    ~Astar();

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

    double getH(cv::Point2i p);
};
