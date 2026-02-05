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
class PiecewiseJerkSmoother2
{
public:
    PiecewiseJerkSmoother2() = delete;
    PiecewiseJerkSmoother2(const std::array<double, 4> & lateral_weights, const std::array<double, 3> & end_state_weights,
                           const double dl_limit, const double vehicle_kappa_max)
                        :  weight_l_(lateral_weights[0]), weight_dl_(lateral_weights[1]),
                           weight_ddl_(lateral_weights[2]), weight_dddl_(lateral_weights[3]),
                           weight_end_state_(end_state_weights),
                           dl_limit_(dl_limit), vehicle_kappa_max_(vehicle_kappa_max) {}
    PiecewiseJerkSmoother2(const PiecewiseJerkSmoother2 &) = delete;
    PiecewiseJerkSmoother2(PiecewiseJerkSmoother2 &&) = delete;
    PiecewiseJerkSmoother2 & operator=(const PiecewiseJerkSmoother2 &) = delete;
    PiecewiseJerkSmoother2 & operator=(PiecewiseJerkSmoother2 &&) = delete;
    ~PiecewiseJerkSmoother2() = default;

    bool Solve(const Path::ReferencePath::Ptr & ref_path, const std::vector<std::array<std::pair<double, double>, 3>> & bounds,
               const std::array<double, 3> & init_state, const std::array<double, 3> & end_state_ref, std::vector<cv::Point2d> & optimized_path) const;

private:
    double weight_l_;
    double weight_dl_;
    double weight_ddl_;
    double weight_dddl_;
    std::array<double, 3> weight_end_state_;

    double dl_limit_;
    double vehicle_kappa_max_;
};
} // namespace Smoother
