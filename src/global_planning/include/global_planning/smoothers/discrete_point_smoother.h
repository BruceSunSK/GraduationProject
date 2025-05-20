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

    /// @brief 针对TSHAstar使用的solve
    /// @param raw_ref_path 原始路径，是一整段的所有原始路径点
    /// @param ref_path 平滑后的路径，所有路径点
    bool Solve(const Path::ReferencePath::Ptr & raw_ref_path, Path::ReferencePath::Ptr & ref_path) const;

    /// @brief 针对QHAstar使用的solve
    /// @param raw_path 原始路径，是一整段的所有原始路径点
    /// @param path 平滑后的路径，所有路径点
    /// @param start_index 针对QHAstar，确定这一单次开始平滑的起点索引，即这个点是会变化的。
    /// @param end_index 针对QHAstar，确定这一单次平滑的终点索引，这个除非是整个路径终点，否则也是会变化的。
    /// @param nb 针对QHAstar，确定这一单次平滑时需要向前固定的节点个数。（注：和原始论文不同，这里把原论文中的那个“单次起点”，“1”也算进去了）
    /// @return 平滑是否成功
    bool Solve(const std::vector<cv::Point2d> & raw_path, std::vector<cv::Point2d> & path,
               const int start_index, const int end_index, const int nb) const;

private:
    std::array<double, 3> weights_;     // 平滑代价权重、长度代价（均匀代价）权重、偏离代价权重
    double buffer_ = 0.0;               // 路径边界缓冲距离
};

} // namespace Smoother
