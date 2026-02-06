#pragma once
#include "global_planning/path/data_type.h"


namespace Vehicle
{
struct State
{
    using Ptr = std::shared_ptr<State>;

    State() : pos(), v(0.0), w(0.0), a(0.0), j(0.0) {}
    State(const Path::PathNode & pos, const double v, const double w, const double a, const double j)
        : pos(pos), v(v), w(w), a(a), j(j) {}

    Path::PathNode pos;
    double v;
    double w;
    double a;
    double j;
};

} // namespace Vehicle
