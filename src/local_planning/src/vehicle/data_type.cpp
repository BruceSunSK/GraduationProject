#include "local_planning/vehicle/data_type.h"  


namespace Vehicle
{
std::ostream & operator<<(std::ostream & os, const State & state)
{
    os << state.pos
        << "(v, w) : (" << state.v << ", " << state.w << ")\n"
        << "(a, j) : (" << state.a << ", " << state.j << ")\n";
    return os;
}

} // namespace Vehicle
