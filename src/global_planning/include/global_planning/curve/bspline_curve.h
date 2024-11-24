#pragma once
#include <vector>

#include <opencv2/core.hpp>


namespace Curve
{
/// @brief B样条曲线。控制点个数为n+1，阶数为k，函数次数为k-1。
class BSplineCurve
{
public:
    BSplineCurve() = delete;
    BSplineCurve(const BSplineCurve & other) = delete;
    BSplineCurve(BSplineCurve && other) = delete;
    BSplineCurve & operator=(const BSplineCurve & other) = delete;
    BSplineCurve & operator=(BSplineCurve && other) = delete;
    ~BSplineCurve() = default;

    /// @brief 使用控制点生成B样条曲线。
    /// @tparam PointType 支持OpenCV格式的整型和浮点型数据点
    /// @param control_points 输入的原始路径点
    /// @param bspline_points 输出的平滑后的稠密序列点
    /// @param k B样条曲线的阶数，生成k-1次函数。通常使用4阶，此时保证曲率平滑。
    /// @param u_step B样条曲线中u的步增长度，t∈[0, 1]，每一步生成一个输出点
    template <typename PointType>
    static void SmoothCurve(const std::vector<cv::Point_<PointType>> & control_points, std::vector<cv::Point2d> & bspline_points, const size_t k = 4, const double u_step = 0.005)
    {
        bspline_points.clear();

        if (control_points.size() == 0)
        {
            return;
        }
        if (control_points.size() == 1)
        {
            bspline_points.push_back(cv::Point2d(control_points.front().x, control_points.front().y));
            return;
        }
        
        const size_t n = control_points.size() - 1;
        const size_t k_ = k > (n + 1) ? (n + 1) : k;    // 阶数不能大于控制点数
        GenerateKnots(k_, n);
        for (double u = 0.0; u <= 1.0; u += u_step)
        {
            Bik_u(u, k_, n);
            cv::Point2d pt(0.0, 0.0);
            for (size_t i = 0; i <= n; i++)
            {
                const double & coff = b_matrix_[k_ - 1][i];
                pt.x += (coff * control_points[i].x);
                pt.y += (coff * control_points[i].y);
            }
            bspline_points.push_back(std::move(pt));
        }
    }

private:
    static std::vector<double> knots_;                  // 记录knots的表，大小为n+k+1
    static std::vector<std::vector<double>> b_matrix_;  // 记录B矩阵的表，第0行大小为n+k，第i行大小为n+k-i，只记录当前u所在区间的B值

    /// @brief 生成准均匀B样条的knots
    /// @param k 阶数
    /// @param n 控制点数-1
    /// @param min 最小值
    /// @param max 最大值
    static void GenerateKnots(const size_t k, const size_t n, const double min = 0.0, const double max = 1.0);

    /// @brief 用于生成在给定的u下，所有的Bik的计算值，储存在表中，每次调用都会重新生成。
    /// @param u 当前的u值。
    /// @param k 阶数，即生成k阶B样条曲线，k-1次函数
    /// @param n 控制点数-1
    static void Bik_u(const double u, const size_t k, const size_t n);
};

} // namespace Curve
