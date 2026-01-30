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

struct TrajectoryPoint : public PointXY
{
    TrajectoryPoint() : PointXY(), t(0), v(0), a(0), j(0) {}
    TrajectoryPoint(const double x, const double y, const double t, const double v, const double a, const double j)
        : PointXY(x, y), t(t), v(v), a(a), j(j) {}
    
    double t;
    double v;
    double a;
    double j;
};

} // namespace Path
