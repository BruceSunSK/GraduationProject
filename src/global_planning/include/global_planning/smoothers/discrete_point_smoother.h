#pragma once
#include <vector>
#include <array>

#include <opencv2/core.hpp>

#include <Eigen/Core>
#include <Eigen/Sparse>

#include <OsqpEigen/OsqpEigen.h>

#include "global_planning/path/reference_path.h"


namespace Smoother
{
class DiscretePointSmoother
{
public:
    DiscretePointSmoother() = delete;
    DiscretePointSmoother(const std::array<double, 3> & weights, const double buffer) : weights_(weights), buffer_(buffer) {}
    DiscretePointSmoother(const DiscretePointSmoother & other) = delete;
    DiscretePointSmoother(DiscretePointSmoother && other) = delete;
    DiscretePointSmoother & operator=(const DiscretePointSmoother & other) = delete;
    DiscretePointSmoother & operator=(DiscretePointSmoother && other) = delete;
    ~DiscretePointSmoother() = default;

    bool Solve(const Path::ReferencePath::Ptr & raw_ref_path, Path::ReferencePath::Ptr & ref_path) const;

private:
    std::array<double, 3> weights_;     // 平滑代价权重、长度代价（均匀代价）权重、偏离代价权重
    double buffer_ = 0.0;               // 路径边界缓冲距离
};

} // namespace Smoother
