#pragma once
#include <iostream>
#include <vector>
#include <queue>
#include <algorithm>

#include "my_astar/global_planning_interface.h"

class Astar : public GlobalPlannerInterface
{
    enum class HeuristicsType
    {
        None, 
        Manhattan, 
        Euclidean
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
        float cost = 0;

        float g = 0;
        float h = 0;
        float f = 0;
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
    bool getSmoothPath(std::vector<cv::Point2f> & path) override;

private:
    std::vector<std::vector<Node>> map_;
    Node * start_node_;
    Node * end_node_;

    int init_map_;
    int init_start_node_;
    int init_end_node_;
    HeuristicsType type_;

    float getH(cv::Point2i p);
};
