#pragma once
#include <vector>
#include <memory>

#include <opencv2/core.hpp>

#include "global_planning/tools/math.h"
#include "global_planning/curve/cubic_spline.h"
#include "global_planning/path/data_type.h"


namespace Path
{
/// @brief 用于表示参考路径的类。在给定原始离散点并指定离散间隔s_interval后，会生成一系列距离均匀的路点。
/// 后续可以利用优化对该路径进行优化，得到更加平滑的曲线。
class ReferencePath
{
public:
    using Ptr = std::shared_ptr<ReferencePath>;

public:
    ReferencePath() = delete;
    ReferencePath(const std::vector<cv::Point2d> & path, const double s_interval)
    {
        SetPath(path, s_interval);
    }
    ReferencePath(const ReferencePath & other) = delete;
    ReferencePath(ReferencePath && other) = delete;
    ReferencePath & operator=(const ReferencePath & other) = delete;
    ReferencePath & operator=(ReferencePath && other) = delete;
    ~ReferencePath() = default;

    void SetPath(const std::vector<cv::Point2d> & path, const double s_interval);
    std::vector<cv::Point2d> GetPath() const;
    PathNode GetPathNode(const double s) const;
    const std::vector<PathNode> & GetPathNodes() const { return path_; }
    double GetSInterval() const { return s_interval_; }
    double GetLength() const { return path_.back().s; }
    size_t GetSize() const { return path_.size(); }

private:
    std::vector<PathNode> path_;
    double s_interval_;
    Curve::CubicSpline X_S_;   // 自然边界曲线
    Curve::CubicSpline Y_S_;
};


} // namespace Path
