#include "global_planning/path/data_type.h"


namespace Path
{
std::ostream & operator<<(std::ostream & os, const PathNode & point)
{
    os  << "(x, y) : (" << point.x << ", " << point.y << ")\n"
        << "(s, l) : (" << point.s << ", " << point.l << ")\n"
        << "(theta, kappa, dkappa) : (" << point.theta << ", " << point.kappa << ", " << point.dkappa << ")\n";
    return os;
}

} // namespace path