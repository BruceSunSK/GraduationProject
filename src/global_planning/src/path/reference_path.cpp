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
    std::list<double> s_list;
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
        const double dddx = X_S_(s, 3);
        const double dddy = Y_S_(s, 3);
        node.theta = Math::Heading(dx, dy);
        node.kappa = Math::Curvature(dx, dy, ddx, ddy);
        node.dkappa = Math::CurvatureRate(dx, dy, ddx, ddy, dddx, dddy);
        path_.push_back(std::move(node));
    }
}

ReferencePath::Ptr ReferencePath::GetSegment(const double start_s, const double end_s, const double s_interval) const
{
    std::vector<cv::Point2d> points;
    double s = std::max(start_s, 0.0);
    double end_s_ = std::min(end_s, GetLength());
    while (s < end_s_)
    {
        points.emplace_back(X_S_(s), Y_S_(s));
        s += s_interval;
    }
    points.emplace_back(X_S_(end_s_), Y_S_(end_s_));
    return std::make_shared<ReferencePath>(points, s_interval);
}

std::pair<PathNode, int> ReferencePath::GetProjection(
    const PointXY & point, const int warmup_start_idx) const
{
    /// @brief 将XY点投影到参考线上，返回SL坐标
    /// @param point 输入的XY点
    /// @param warmup_start_idx 起始搜索下标，用于warmup
    /// @return 对应的投影点，以及最近点的下标
    /// 算法步骤：
    /// 1. 首先在离散的path_中找到距离给定点最近的点，使用warmup_start_idx作为起始搜索下标
    /// 2. 使用牛顿法迭代优化，找到精确的投影点
    /// 3. 计算横向偏移l（有正负，表示在参考线的左侧还是右侧）

    // 步骤1：在离散的path_中找到距离给定点最近的点，从warmup_start_idx开始搜索
    // 确保warmup_start_idx在有效范围内
    int start_idx = Math::Clamp<int>(warmup_start_idx, 0, path_.size() - 1);
    double min_dis_sq = std::numeric_limits<double>::max();
    int nearest_idx = start_idx;

    // 向前搜索（从start_idx到末尾）
    for (int i = start_idx; i < path_.size(); ++i)
    {
        double dx = point.x - path_[i].x;
        double dy = point.y - path_[i].y;
        double dis_sq = dx * dx + dy * dy;

        if (dis_sq < min_dis_sq)
        {
            min_dis_sq = dis_sq;
            nearest_idx = i;
        }
    }

    // 获取最近点的s值作为初始值
    double s0 = path_[nearest_idx].s;

    // 步骤2：使用牛顿法迭代优化，找到精确的投影点
    const int max_iterations = 8;
    const double tolerance = 1e-6;
    double s = s0;

    for (int iter = 0; iter < max_iterations; ++iter)
    {
        // 计算当前位置的值和导数
        double x = X_S_(s);
        double y = Y_S_(s);
        double dx = X_S_(s, 1);  // 一阶导数
        double dy = Y_S_(s, 1);
        double ddx = X_S_(s, 2); // 二阶导数
        double ddy = Y_S_(s, 2);

        // 计算点到曲线的向量
        double fx = x - point.x;
        double fy = y - point.y;

        // 计算函数值：f(s) = (r(s) - p)·r'(s)
        double f = fx * dx + fy * dy;

        // 计算导数值：f'(s) = |r'(s)|² + (r(s) - p)·r''(s)
        double df = dx * dx + dy * dy + fx * ddx + fy * ddy;

        // 避免除零
        if (std::abs(df) < 1e-10)
        {
            break;
        }

        // 牛顿法更新：s_{n+1} = s_n - f(s_n)/f'(s_n)
        double delta_s = -f / df;
        s += delta_s;

        // 确保s在有效范围内 [0, length]
        s = Math::Clamp(s, 0.0, GetLength());

        // 检查收敛
        if (std::abs(delta_s) < tolerance)
        {
            break;
        }
    }

    // 确保s在有效范围内
    s = Math::Clamp(s, 0.0, GetLength());

    // 步骤3：计算横向偏移l
    // 获取投影点的位置和朝向
    double x_proj = X_S_(s);
    double y_proj = Y_S_(s);
    double dx_proj = X_S_(s, 1);
    double dy_proj = Y_S_(s, 1);

    // 计算点到投影点的向量
    double dx_point = point.x - x_proj;
    double dy_point = point.y - y_proj;

    // 计算参考线的切向量
    double heading = Math::Heading(dx_proj, dy_proj);

    // 计算横向偏移l：点到投影点的向量在法向上的投影
    // 法向向量为：(-sin(heading), cos(heading))
    double l = -dx_point * std::sin(heading) + dy_point * std::cos(heading);

    // 返回投影点
    PathNode node;
    node.x = x_proj;
    node.y = y_proj;
    node.s = s;
    node.l = l;
    const double ddx_proj = X_S_(s, 2);
    const double ddy_proj = Y_S_(s, 2);
    const double dddx_proj = X_S_(s, 3);
    const double dddy_proj = Y_S_(s, 3);
    node.theta = heading;
    node.kappa = Math::Curvature(dx_proj, dy_proj, ddx_proj, ddy_proj);
    node.dkappa = Math::CurvatureRate(dx_proj, dy_proj, ddx_proj, ddy_proj, dddx_proj, dddy_proj);

    return { node, nearest_idx };
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
    const double dddx = X_S_(s, 3);
    const double dddy = Y_S_(s, 3);
    node.theta = Math::Heading(dx, dy);
    node.kappa = Math::Curvature(dx, dy, ddx, ddy);
    node.dkappa = Math::CurvatureRate(dx, dy, ddx, ddy, dddx, dddy);
    return node;
}

} // namespace Path