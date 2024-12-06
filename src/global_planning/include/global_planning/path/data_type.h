#pragma once


namespace Path
{
struct PointXY
{
    PointXY() : x(0), y(0) {}
    PointXY(const double x, const double y) : x(x), y(y) {}

    double x;
    double y;
};

struct PointSL
{
    PointSL() : s(0), l(0) {}
    PointSL(const double s, const double l) : s(s), l(l) {}

    double s;
    double l;
};

struct PathNode : public PointXY, public PointSL
{
    PathNode() : PointXY(), PointSL() {}
    PathNode(const double x, const double y, const double s, const double l) : PointXY(x, y), PointSL(s, l) {}
    PathNode(const double x, const double y, const double s, const double l, const double theta, const double kappa)
        : PointXY(x, y), PointSL(s, l), theta(theta), kappa(kappa) {}

    double theta;
    double kappa;
};

} // namespace Path
