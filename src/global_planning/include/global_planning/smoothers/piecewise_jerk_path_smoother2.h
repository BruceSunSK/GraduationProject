#pragma once
#include <vector>
#include <array>

#include <opencv2/core.hpp>
#include <Eigen/Core>
#include <Eigen/Sparse>
#include <OsqpEigen/OsqpEigen.h>

#include "global_planning/path/reference_path.h"
#include "global_planning/path/utils.h"


namespace Smoother
{
/// @brief 基本与PiecewiseJerkSmoother一致，区别点：①加入了三碰撞圆的障碍物约束②不需要居中约束
class PiecewiseJerkPathSmoother2
{
public:
    struct Weights
    {
        double w_l;
        double w_dl;
        double w_ddl;
        double w_dddl;
        double w_end_state_l;
        double w_end_state_dl;
        double w_end_state_ddl;
    };

    struct Params
    {
        double dl_limit;
        double vehicle_kappa_max;
    };

public:
    PiecewiseJerkPathSmoother2() = delete;
    PiecewiseJerkPathSmoother2(const Weights & weights, const Params & params) : weights_(weights), params_(params) {}
    PiecewiseJerkPathSmoother2(const PiecewiseJerkPathSmoother2 &) = delete;
    PiecewiseJerkPathSmoother2(PiecewiseJerkPathSmoother2 &&) = delete;
    PiecewiseJerkPathSmoother2 & operator=(const PiecewiseJerkPathSmoother2 &) = delete;
    PiecewiseJerkPathSmoother2 & operator=(PiecewiseJerkPathSmoother2 &&) = delete;
    ~PiecewiseJerkPathSmoother2() = default;

    bool Solve(const double ds, const std::vector<Path::PathNode> & ref_points,
               const std::vector<std::array<std::pair<double, double>, 3>> & bounds,
               const std::array<double, 3> & init_state, const std::array<double, 3> & end_state_ref, 
               std::vector<Path::PointXY> & optimized_path) const;

private:
    Weights weights_;
    Params params_;
};
} // namespace Smoother
