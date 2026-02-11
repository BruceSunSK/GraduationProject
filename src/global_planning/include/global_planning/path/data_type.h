#pragma once
#include <iostream>


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

struct PointSLWithDerivatives : public PointSL
{
    PointSLWithDerivatives() : PointSL(), l_prime(0), l_double_prime(0) {}
    PointSLWithDerivatives(const double s, const double l, const double l_prime, const double l_double_prime)
        : PointSL(s, l), l_prime(l_prime), l_double_prime(l_double_prime) {}
    PointSLWithDerivatives(const PointSL & point, const double l_prime, const double l_double_prime)
        : PointSL(point), l_prime(l_prime), l_double_prime(l_double_prime){}

    double l_prime;
    double l_double_prime;
};

struct PathNode : public PointXY, public PointSL
{
    PathNode() : PointXY(), PointSL() {}
    PathNode(const double x, const double y, const double s, const double l) : PointXY(x, y), PointSL(s, l) {}
    PathNode(const double x, const double y, const double s, const double l, const double theta, const double kappa, double dkappa)
        : PointXY(x, y), PointSL(s, l), theta(theta), kappa(kappa), dkappa(dkappa) {}

    double theta;
    double kappa;
    double dkappa;
};
std::ostream & operator<<(std::ostream & os, const PathNode & point);

struct TrajectoryPoint
{
    TrajectoryPoint() : x(0), y(0), theta(0), kappa(0), v(0), a(0), j(0), s(0), l(0), t(0) {}

    double x;
    double y;
    double theta;  // 航向角
    double kappa;  // 曲率
    double v;      // 速度
    double a;      // 加速度
    double j;      // 加加速度
    double s;      // 路径长度
    double l;      // 车辆位置
    double t;      // 时间戳（相对于轨迹起点）
};

} // namespace Path
