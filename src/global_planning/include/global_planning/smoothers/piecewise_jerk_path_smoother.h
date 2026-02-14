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
class PiecewiseJerkPathSmoother
{
public:
    struct Weights
    {
        double w_l;
        double w_dl;
        double w_ddl;
        double w_dddl;
        double w_center;
        double w_end_state_l;
        double w_end_state_dl;
        double w_end_state_ddl;
    };

    struct Params
    {
        double lateral_sample_range;
        double dl_limit;
        double vehicle_kappa_max;
        double center_deviation_thres;
        double center_bounds_thres;
        double center_obs_coeff;
    };
    
public:
    PiecewiseJerkPathSmoother() = delete;
    PiecewiseJerkPathSmoother(const Weights & weights, const Params & params) : weights_(weights), params_(params) {}
    PiecewiseJerkPathSmoother(const PiecewiseJerkPathSmoother&) = delete;
    PiecewiseJerkPathSmoother(PiecewiseJerkPathSmoother &&) = delete;
    PiecewiseJerkPathSmoother & operator=(const PiecewiseJerkPathSmoother &) = delete;
    PiecewiseJerkPathSmoother & operator=(PiecewiseJerkPathSmoother &&) = delete;
    ~PiecewiseJerkPathSmoother() = default;

    bool Solve(const Path::ReferencePath::Ptr & raw_ref_path, const std::vector<std::pair<double, double>> & bounds,
               const std::array<double, 3> & init_state, const std::array<double, 3> & end_state_ref, std::vector<cv::Point2d> & optimized_path) const;

private:
    Weights weights_;
    Params params_;
};
} // namespace Smoother
