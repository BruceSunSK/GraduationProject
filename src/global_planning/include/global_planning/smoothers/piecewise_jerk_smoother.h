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
class PiecewiseJerkSmoother
{
public:
    PiecewiseJerkSmoother() = delete;
    PiecewiseJerkSmoother(const std::array<double, 4> & lateral_weights, const double center_weight, const std::array<double, 3> & end_state_weights,
                          const double lateral_sample_range, const double dl_limit, const double vehicle_kappa_max,
                          const double center_deviation_thres, const double center_bounds_thres, const double center_obs_coeff)
                        : weight_l_(lateral_weights[0]), weight_dl_(lateral_weights[1]),
                          weight_ddl_(lateral_weights[2]), weight_dddl_(lateral_weights[3]),
                          weight_center_(center_weight), weight_end_state_(end_state_weights),
                          lateral_sample_range_(lateral_sample_range), dl_limit_(dl_limit), vehicle_kappa_max_(vehicle_kappa_max),
                          center_deviation_thres_(center_deviation_thres), center_bounds_thres_(center_bounds_thres), center_obs_coeff_(center_obs_coeff) {}
    PiecewiseJerkSmoother(const PiecewiseJerkSmoother&) = delete;
    PiecewiseJerkSmoother(PiecewiseJerkSmoother &&) = delete;
    PiecewiseJerkSmoother & operator=(const PiecewiseJerkSmoother &) = delete;
    PiecewiseJerkSmoother & operator=(PiecewiseJerkSmoother &&) = delete;
    ~PiecewiseJerkSmoother() = default;

    bool Solve(const Path::ReferencePath::Ptr & raw_ref_path, const std::vector<std::pair<double, double>> & bounds,
               const std::array<double, 3> & init_state, const std::array<double, 3> & end_state_ref, std::vector<cv::Point2d> & optimized_path) const;

private:
    double weight_l_;
    double weight_dl_;
    double weight_ddl_;
    double weight_dddl_;
    double weight_center_;
    std::array<double, 3> weight_end_state_;

    double lateral_sample_range_;
    double dl_limit_;
    double vehicle_kappa_max_;
    double center_deviation_thres_;
    double center_bounds_thres_;
    double center_obs_coeff_;
};
} // namespace Smoother
