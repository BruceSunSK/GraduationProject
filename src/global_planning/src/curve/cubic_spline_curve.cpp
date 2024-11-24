#include "global_planning/curve/cubic_spline_curve.h"


namespace Curve
{
double CubicSplineCurve::operator()(const double x, const int derivative) const
{
    std::vector<cv::Point2d>::const_iterator it = std::lower_bound(points_.begin(), points_.end(), x,
        [](const cv::Point2d & p, const double & x) -> bool
        {
            return p.x < x;
        });

    const size_t idx = std::distance(points_.begin(), it);
    const double h = x - points_[idx == 0 ? 0 : idx - 1].x;

    switch (derivative)
    {
    case 0:
        return ((coeffs_(idx, 0) * h + coeffs_(idx, 1)) * h + coeffs_(idx, 2)) * h + coeffs_(idx, 3);
    case 1:
        return (3.0 * coeffs_(idx, 0) * h + 2.0 * coeffs_(idx, 1)) * h + coeffs_(idx, 2);
    case 2:
        return 6.0 * coeffs_(idx, 0) * h + 2.0 * coeffs_(idx, 1);
    case 3:
        return 6.0 * coeffs_(idx, 0);
    default:
        return 0.0;
    }
}


void CubicSplineCurve::CalculateCoefficients()
{
    size_t n = points_.size();
    assert(n > 2);

    Eigen::SparseMatrix<double> A_; // 系数矩阵
    Eigen::VectorXd b_;             // 右端常数
    A_.resize(n, n);
    A_.setZero();
    b_.resize(n);
    b_.setZero();

    // 填补中心已知内容
    for (size_t i = 1; i < n - 1; i++)
    {
        const cv::Point2d & p0 = points_[i - 1];
        const cv::Point2d & p1 = points_[i];
        const cv::Point2d & p2 = points_[i + 1];
        const cv::Point2d dp0 = p1 - p0;
        const cv::Point2d dp1 = p2 - p1;

        A_.insert(i, i) = 2.0 * (p2.x - p0.x);
        A_.insert(i, i - 1) = dp0.x;
        A_.insert(i, i + 1) = dp1.x;
        b_(i) = 6 * (dp1.y / dp1.x - dp0.y / dp0.x);
    }

    // 填补边界内容
    switch (left_bc_)
    {
    case BoundaryCondition::First_Derive:
    {
        const cv::Point2d dp0 = points_[1] - points_[0];
        A_.insert(0, 0) = 2.0 * dp0.x;
        A_.insert(0, 1) = dp0.x;
        b_(0) = 6.0 * (dp0.y / dp0.x - left_value_);
        break;
    }
    case BoundaryCondition::Second_Derive:
    {
        A_.insert(0, 0) = 1.0;
        b_(0) = left_value_;
        break;
    }
    default:
        break;
    }
    switch (right_bc_)
    {
    case BoundaryCondition::First_Derive:
    {
        const cv::Point2d dpn_2 = points_[n - 1] - points_[n - 2];
        A_.insert(n - 1, n - 1) = 2.0 * dpn_2.x;
        A_.insert(n - 1, n - 2) = dpn_2.x;
        b_(n - 1) = 6.0 * (right_value_ - dpn_2.y / dpn_2.x);
        break;
    }
    case BoundaryCondition::Second_Derive:
    {
        A_.insert(n - 1, n - 1) = 1.0;
        b_(n - 1) = right_value_;
        break;
    }
    default:
        break;
    }

    // 求解矩阵
    Eigen::SparseLU<Eigen::SparseMatrix<double>> solver;
    solver.compute(A_);
    Eigen::VectorXd m_ = solver.solve(b_);

    // 计算系数
    coeffs_.resize(n + 1, 4);      // [1, n - 1]共n - 1段，记录中间系数，0上记录左端，n上记录右端
    for (size_t i = 1; i < n; ++i)
    {
        const cv::Point2d dpi_1 = points_[i] - points_[i - 1];
        // 三次项系数 a
        coeffs_(i, 0) = (m_[i] - m_[i - 1]) / (6.0 * dpi_1.x);
        // 二次项系数 b
        coeffs_(i, 1) = m_[i - 1] * 0.5;
        // 一阶项系数 c
        coeffs_(i, 2) = dpi_1.y / dpi_1.x - (m_[i - 1] / 3.0 + m_[i] / 6.0) * dpi_1.x;
        // 常数项系数 d
        coeffs_(i, 3) = points_[i - 1].y;
    }

    // 计算左右两侧的外推部分的系数
    switch (left_bc_)
    {
    case BoundaryCondition::First_Derive:   // 使用线性外推
    {
        coeffs_(0, 0) = 0.0;
        coeffs_(0, 1) = 0.0;
        coeffs_(0, 2) = left_value_;
        coeffs_(0, 3) = points_.front().y;
        break;
    }
    case BoundaryCondition::Second_Derive:  // 使用二次外推
    {
        coeffs_(0, 0) = 0.0;
        coeffs_(0, 1) = coeffs_(1, 1);
        coeffs_(0, 2) = coeffs_(1, 2);
        coeffs_(0, 3) = points_.front().y;
        break;
    }
    default:
        break;
    }
    switch (right_bc_)
    {
    case BoundaryCondition::First_Derive:   // 使用线性外推
    {
        coeffs_(n, 0) = 0.0;
        coeffs_(n, 1) = 0.0;
        coeffs_(n, 2) = right_value_;
        coeffs_(n, 3) = points_.back().y;
        break;
    }
    case BoundaryCondition::Second_Derive:  // 使用二次外推
    {
        const double h = points_[n - 1].x - points_[n - 2].x;
        coeffs_(n, 0) = 0.0;
        coeffs_(n, 1) = m_[n - 1] * 0.5;
        coeffs_(n, 2) = 3.0 * coeffs_(n - 1, 0) * h * h + 2.0 * coeffs_(n - 1, 1) * h + coeffs_(n - 1, 2);
        coeffs_(n, 3) = points_.back().y;
        break;
    }
    default:
        break;
    }
}

} // namespace Curve
