#include "global_planning/tools/bezier_curve.h"


std::vector<std::vector<size_t>> BezierCurve::combination_table_ = {{1}, {1, 1}, {1, 2, 1}, {1, 3, 3, 1}};
