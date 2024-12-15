#include "global_planning/curve/dubins.h"


namespace Curve
{
Dubins::DubinsPath Dubins::Path(const std::array<double, 3> & start, const std::array<double, 3> & end)
{
    const double dx = end[0] - start[0];
    const double dy = end[1] - start[1];

    d_ = std::hypot(dx, dy) / r_;           // 起点到终点归一化后的距离
    theta_ = std::atan2(dy, dx);            // 起点到终点的角度
    alpha_ = Mod2Pi(start[2] - theta_);     // 起点归一化后的朝向角度
    beta_  = Mod2Pi(end[2] - theta_);       // 终点归一化后的朝向角度

    ca_ = std::cos(alpha_);
    sa_ = std::sin(alpha_);
    cb_ = std::cos(beta_);
    sb_ = std::sin(beta_);

    if (d_ < EPS && std::abs(alpha_ - beta_) < EPS)
    {
        return DubinsPath();
    }

    DubinsPath paths[6] = { LSL(), RSR(), LSR(), RSL(), RLR(), LRL() };
    DubinsPath * path = std::min_element(paths, paths + 6, [](const DubinsPath & a, const DubinsPath & b) { return a.Length() < b.Length(); });
    return *path;
}

std::vector<std::array<double, 3>> Dubins::SegmentPath(const DubinsPath & path, const std::array<double, 3> & start) const
{
    const double length = path.Length();    // 归一化的路径总长度
    const double step = step_ / r_;         // 步长归一化后的距离
    std::vector<std::array<double, 3>> points;
    points.reserve(std::ceil(length / step) + 1);
    for (double seg = 0.0; seg < length; seg += step)
    {
        points.emplace_back(Interpolate(path, start, seg));
    }
    return points;
}

Dubins::DubinsPath Dubins::LSL() const
{
    const double tmp = 2.0 + d_ * d_ - 2.0 * (ca_ * cb_ + sa_ * sb_ - d_ * (sa_ - sb_));
    if (tmp < ZERO)
    {
        return {INF, INF, INF, {DubinsSegmentType::L, DubinsSegmentType::S, DubinsSegmentType::L}, INF};
    }

    const double th = std::atan2(cb_ - ca_, d_ + sa_ - sb_);
    const double t = Mod2Pi(-alpha_ + th);
    const double p = std::sqrt(std::max(tmp, 0.0));
    const double q = Mod2Pi(beta_ - th);
    return { t, p, q, {DubinsSegmentType::L, DubinsSegmentType::S, DubinsSegmentType::L}, r_ };
}

Dubins::DubinsPath Dubins::RSR() const
{
    const double tmp = 2.0 + d_ * d_ - 2.0 * (ca_ * cb_ + sa_ * sb_ - d_ * (sb_ - sa_));
    if (tmp < ZERO)
    {
        return { INF, INF, INF, {DubinsSegmentType::R, DubinsSegmentType::S, DubinsSegmentType::R}, INF };
    }
    
    const double th = std::atan2(ca_ - cb_, d_ - sa_ + sb_);
    const double t = Mod2Pi(alpha_ - th);
    const double p = std::sqrt(std::max(tmp, 0.));
    const double q = Mod2Pi(-beta_ + th);
    return { t, p, q, {DubinsSegmentType::R, DubinsSegmentType::S, DubinsSegmentType::R}, r_ };
}

Dubins::DubinsPath Dubins::LSR() const
{
    const double tmp = -2.0 + d_ * d_ + 2.0 * (ca_ * cb_ + sa_ * sb_ + d_ * (sa_ + sb_));
    if (tmp < ZERO)
    {
        return { INF, INF, INF, {DubinsSegmentType::L, DubinsSegmentType::S, DubinsSegmentType::R}, INF };
    }

    const double p = std::sqrt(std::max(tmp, 0.));
    const double th = std::atan2(-ca_ - cb_, d_ + sa_ + sb_) - std::atan2(-2., p);
    const double t = Mod2Pi(-alpha_ + th);
    const double q = Mod2Pi(-beta_ + th);
    return { t, p, q, {DubinsSegmentType::L, DubinsSegmentType::S, DubinsSegmentType::R}, r_ };
}

Dubins::DubinsPath Dubins::RSL() const
{
    const double tmp = d_ * d_ - 2.0 + 2.0 * (ca_ * cb_ + sa_ * sb_ - d_ * (sa_ + sb_));
    if (tmp < ZERO)
    {
        return { INF, INF, INF, {DubinsSegmentType::R, DubinsSegmentType::S, DubinsSegmentType::L}, INF };
    }
    
    const double p = std::sqrt(std::max(tmp, 0.));
    const double th = std::atan2(ca_ + cb_, d_ - sa_ - sb_) - std::atan2(2., p);
    const double t = Mod2Pi(alpha_ - th);
    const double q = Mod2Pi(beta_ - th);
    return { t, p, q, {DubinsSegmentType::R, DubinsSegmentType::S, DubinsSegmentType::L}, r_ };
}

Dubins::DubinsPath Dubins::RLR() const
{
    const double tmp = 0.125 * (6.0 - d_ * d_ + 2.0 * (ca_ * cb_ + sa_ * sb_ + d_ * (sa_ - sb_)));
    if (std::fabs(tmp) > 1.0)
    {
        return { INF, INF, INF, {DubinsSegmentType::R, DubinsSegmentType::L, DubinsSegmentType::R}, INF };
    }

    const double p = 2 * M_PI - std::acos(tmp);
    const double th = std::atan2(ca_ - cb_, d_ - sa_ + sb_);
    const double t = Mod2Pi(alpha_ - th + .5 * p);
    const double q = Mod2Pi(alpha_ - beta_ - t + p);
    return { t, p, q, {DubinsSegmentType::R, DubinsSegmentType::L, DubinsSegmentType::R}, r_ };
}

Dubins::DubinsPath Dubins::LRL() const
{
    const double tmp = 0.125 * (6.0 - d_ * d_ + 2.0 * (ca_ * cb_ + sa_ * sb_ - d_ * (sa_ - sb_)));
    if (std::fabs(tmp) > 1.0)
    {
        return { INF, INF, INF, {DubinsSegmentType::L, DubinsSegmentType::R, DubinsSegmentType::L}, INF };
    }
    
    const double p = 2 * M_PI - std::acos(tmp);
    const double th = std::atan2(-ca_ + cb_, d_ + sa_ - sb_);
    const double t = Mod2Pi(-alpha_ + th + .5 * p);
    const double q = Mod2Pi(beta_ - alpha_ - t + p);
    return { t, p, q, {DubinsSegmentType::L, DubinsSegmentType::R, DubinsSegmentType::L}, r_};
}

std::array<double, 3> Dubins::Interpolate(const DubinsPath & path, const std::array<double, 3> & start, const double s) const
{
    double seg = s;
    if (s < 0.0)
        seg = 0.0;
    if (s > path.Length())
        seg = path.Length();

    std::array<double, 3> out = { 0.0, 0.0, start[2] };
    for (int i = 0; i < 3 && seg > 0; ++i)
    {
        const double v = std::min(seg, path.l[i]);
        const double phi = out[2];

        seg -= v;
        switch (path.type[i])
        {
        case DubinsSegmentType::L:
            out[0] += ( std::sin(phi + v) - std::sin(phi));
            out[1] += (-std::cos(phi + v) + std::cos(phi));
            out[2] = phi + v;
            break;
        case DubinsSegmentType::R:
            out[0] += (-std::sin(phi - v) + std::sin(phi));
            out[1] += ( std::cos(phi - v) - std::cos(phi));
            out[2] = phi - v;
            break;
        case DubinsSegmentType::S:
            out[0] += (v * std::cos(phi));
            out[1] += (v * std::sin(phi));
            break;
        }
    }

    out[0] = out[0] * r_ + start[0];
    out[1] = out[1] * r_ + start[1];
    return out;
}
} // namespace Curve
