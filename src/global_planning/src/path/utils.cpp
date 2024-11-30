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

PointXY SLtoXY(const PointSL sl, const PointXY ref_xy, const double ref_theta)
{
    PointXY xy;
    xy.x = ref_xy.x - sl.l * std::sin(ref_theta);
    xy.y = ref_xy.y + sl.l * std::cos(ref_theta);
    return xy;
}

} // namespace Utils
} // namespace Path
