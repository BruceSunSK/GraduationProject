#include "global_planning/path/reference_path.h"


namespace Path
{
void ReferencePath::SetPath(const std::vector<cv::Point2d> & path, const double s_interval)
{
    assert(path.size() > 6);
    s_interval_ = s_interval;

    // 构造参考线的参数形式
    double s_sum = 0.0;
    std::vector<cv::Point2d> X_S_points;
    std::vector<cv::Point2d> Y_S_points;
    X_S_points.emplace_back(s_sum, path.front().x);
    Y_S_points.emplace_back(s_sum, path.front().y);
    for (size_t i = 1; i < path.size(); i++)
    {
        s_sum += std::hypot(path[i].x - path[i - 1].x, path[i].y - path[i - 1].y);
        X_S_points.emplace_back(s_sum, path[i].x);
        Y_S_points.emplace_back(s_sum, path[i].y);
    }
    X_S_.SetPoints(std::move(X_S_points));
    Y_S_.SetPoints(std::move(Y_S_points));

    // 根据离散间隔生成s集合，保证终点一定在曲线上，但也会使得曲线超出终点，不过不影响，后续超出的部分也有利于优化
    std::vector<double> s_list;
    s_list.emplace_back(0.0);
    while (s_list.back() < s_sum)
    {
        s_list.emplace_back(s_list.back() + s_interval_);
    }
    if (s_list.back() > s_sum)      // 由于浮点数计算误差，可能导致最后一个s超出终点s，舍去
    {
        s_list.pop_back();
    }
    if (s_list.back() + 1 > s_sum)  // 终点s以内过近的最后一个点，舍去，防止与终点过近重合。此处的1的单位是栅格个数，即容忍的范围是一个分辨率的大小。
    {
        s_list.pop_back();
    }
    s_list.emplace_back(s_sum);     // 终点s

    // 根据s集合生成参考线
    path_.clear();
    for (const double s : s_list)
    {
        PathNode node;
        node.s = s;
        node.l = 0.0;
        node.x = X_S_(s);
        node.y = Y_S_(s);
        const double dx = X_S_(s, 1);
        const double dy = Y_S_(s, 1);
        const double ddx = X_S_(s, 2);
        const double ddy = Y_S_(s, 2);
        node.theta = Math::Heading(dx, dy);
        node.kappa = Math::Curvature(dx, dy, ddx, ddy);
        path_.push_back(std::move(node));
    }
}

std::vector<cv::Point2d> ReferencePath::GetPath() const
{
    std::vector<cv::Point2d> path;
    for (const PathNode & node : path_)
    {
        path.emplace_back(node.x, node.y);
    }
    return path;
}

PathNode ReferencePath::GetPathNode(const double s) const
{
    PathNode node;
    node.s = s;
    node.l = 0.0;
    node.x = X_S_(s);
    node.y = Y_S_(s);
    const double dx = X_S_(s, 1);
    const double dy = Y_S_(s, 1);
    const double ddx = X_S_(s, 2);
    const double ddy = Y_S_(s, 2);
    node.theta = Math::Heading(dx, dy);
    node.kappa = Math::Curvature(dx, dy, ddx, ddy);
    return node;
}

} // namespace Path