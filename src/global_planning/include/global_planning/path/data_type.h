#pragma once

#include <vector>

#include <opencv2/core.hpp>

namespace Path
{
struct PointXY
{
    PointXY() : x(0), y(0) {}
    PointXY(double x, double y) : x(x), y(y) {}
    operator cv::Point2d() const { return cv::Point2d(x, y); }

    double x;
    double y;
};

struct PointSL
{
    PointSL() : s(0), l(0) {}
    PointSL(double s, double l) : s(s), l(l) {}

    double s;
    double l;
};

struct PathNode : public PointXY, public PointSL
{
    PathNode() : PointXY(), PointSL() {}
    PathNode(double x, double y, double s, double l) : PointXY(x, y), PointSL(s, l) {}
    operator cv::Point2d() = delete;

    double yaw;
    double kappa;
    double dkappa;
};

} // namespace Path
