#include "global_planning/path/utils.h"


namespace Path
{
namespace Utils
{
PointSL XYtoSL(const PointXY xy, const PointXY ref_xy, const double ref_s, const double ref_theta)
{
    PointSL sl;
    sl.s = ref_s;
    sl.l = std::hypot(xy.x - ref_xy.x, xy.y - ref_xy.y);
    sl.l *= ((xy.y - ref_xy.y) * std::cos(ref_theta)) > ((xy.x - ref_xy.x) * std::sin(ref_theta) ? 1 : -1);
    return sl;
}

PointSLWithDerivatives XYtoSL(const PointXY xy, const double theta, const double kappa,
                              const PointXY ref_xy, const double ref_s, const double ref_theta,
                              const double ref_kappa, const double ref_kappa_prime)
{
    PointSLWithDerivatives sl;
    sl.s = ref_s;
    sl.l = std::hypot(xy.x - ref_xy.x, xy.y - ref_xy.y);
    sl.l *= ((xy.y - ref_xy.y) * std::cos(ref_theta)) > ((xy.x - ref_xy.x) * std::sin(ref_theta) ? 1 : -1);

    const double dtheta = theta - ref_theta;
    const double cos_dtheta = std::cos(dtheta);
    static const double epsilon = 1e-6;

    sl.l_prime = (1 - ref_kappa * sl.l) * std::tan(dtheta);
    sl.l_double_prime = -(ref_kappa_prime * sl.l + ref_kappa * sl.l_prime) * std::tan(dtheta)
                        + (1 - ref_kappa * sl.l) / (cos_dtheta * cos_dtheta + epsilon)
                            * ((1 - ref_kappa * sl.l) / cos_dtheta * kappa - ref_kappa);
    return sl;
}

PointXY SLtoXY(const PointSL sl, const PointXY ref_xy, const double ref_theta)
{
    PointXY xy;
    xy.x = ref_xy.x - sl.l * std::sin(ref_theta);
    xy.y = ref_xy.y + sl.l * std::cos(ref_theta);
    return xy;
}

PathNode GlobalToLocal(const PathNode & reference, const PathNode & target)
{
    double dx = target.x - reference.x;
    double dy = target.y - reference.y;

    PathNode local;
    local.x = dx * std::cos(reference.theta) + dy * std::sin(reference.theta);
    local.y = -dx * std::sin(reference.theta) + dy * std::cos(reference.theta);
    local.theta = target.theta - reference.theta;
    local.kappa = target.kappa;
    local.dkappa = target.kappa;
    return local;
}
} // namespace Utils
} // namespace Path
