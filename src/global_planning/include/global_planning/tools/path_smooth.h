#pragma once
#include <vector>

#include <opencv2/core.hpp>


/// @brief 贝塞尔曲线。可以用所有控制点生成一段曲线，也可以用分段的三阶贝塞尔曲线进行平滑拟合。
class BezierCurve
{
public:
    BezierCurve() = default;
    ~BezierCurve() = default;

    /// @brief 根据控制点进行贝塞尔平滑。用t_step控制生成点的数目
    /// @tparam PointType 支持OpenCV格式的整型和浮点型数据点
    /// @param control_points 输入的原始控制点
    /// @param bezier_points 输出的平滑后的稠密序列点
    /// @param t_step 贝塞尔平滑中t的步增长度，t∈[0, 1]，每一步生成一个输出点
    template <typename PointType>
    static void smooth_curve(const std::vector<cv::Point_<PointType>> & control_points, std::vector<cv::Point2d> & bezier_points, double t_step = 0.01)
    {
        bezier_points.clear();

        if (control_points.size() == 0)
        {
            return;
        }
        if (control_points.size() == 1)
        {
            bezier_points.push_back(cv::Point2d(control_points.front().x, control_points.front().y));
            return;
        }
        
        const size_t n = control_points.size() - 1;
        for (double t = 0; t <= 1; t += t_step)
        {
            cv::Point2d pt(0.0, 0.0);
            for (size_t i = 0; i <= n; i++)
            {
                const double coff = C_n_r(n, i) * std::pow(t, i) * std::pow(1 - t, n - i);
                pt.x += (coff * control_points[i].x);
                pt.y += (coff * control_points[i].y);
            }
            bezier_points.push_back(std::move(pt));
        }
    }

    /// @brief 将输入的点利用分段的三阶贝塞尔曲线进行平滑拟合。除去起点和终点外，每两个点一组，每组间插值生成中间控制节点，每组共有4个可用控制点，生成三阶贝塞尔曲线。
    ///        然后插值生成中间控制节点，以保证路径平滑。用t_step控制生成点的数目
    /// @tparam PointType 支持OpenCV格式的整型和浮点型数据点
    /// @param input_points 输入的原始路径点
    /// @param bezier_points 输出的平滑后的稠密序列点
    /// @param t_step 贝塞尔平滑中t的步增长度，t∈[0, 1]，每一步生成一个输出点
    template <typename PointType>
    static void piecewise_smooth_curve(const std::vector<cv::Point_<PointType>> & input_points, std::vector<cv::Point2d> & bezier_points, double t_step = 0.01)
    {
        bezier_points.clear();
        const size_t size = input_points.size();
        if (size <= 4)  // 4个点以下就不分段，直接拟合。
        {
            smooth_curve(input_points, bezier_points, t_step);
            return;
        }
        
        const int part_nums = (size - 2) / 2;
        std::vector<cv::Point2d> sub_points;
        sub_points.push_back(cv::Point2d(input_points.front().x, input_points.front().y)); // 起点
        for (int i = 0; i < part_nums - 1; i++) // 靠近终点的最后一段特殊处理
        {
            // 中间两节点
            const int sub_index = 1 + i * 2;
            sub_points.push_back(cv::Point2d(input_points[sub_index].x, input_points[sub_index].y));
            sub_points.push_back(cv::Point2d(input_points[sub_index + 1].x, input_points[sub_index + 1].y));

            // 插入的新节点作为最后一个节点
            cv::Point2d pi;
            pi.x = (input_points[sub_index + 1].x + input_points[sub_index + 2].x) / 2.0;
            pi.y = (input_points[sub_index + 1].y + input_points[sub_index + 2].y) / 2.0;
            sub_points.push_back(pi);
            
            // 生成该段的三阶贝塞尔曲线
            std::vector<cv::Point2d> sub_smooth_points;
            smooth_curve(sub_points, sub_smooth_points, t_step);
            bezier_points.insert(bezier_points.end(), sub_smooth_points.begin(), sub_smooth_points.end());

            // 插入的新节点也将作为下一段贝塞尔曲线的第一个节点
            sub_points.clear();
            sub_points.push_back(std::move(pi));
        }

        // 最后一段特殊处理，可能是三阶贝塞尔，也可能是二阶贝塞尔
        sub_points.insert(sub_points.end(), input_points.begin() + part_nums * 2 - 1, input_points.end());
        std::vector<cv::Point2d> sub_smooth_points;
        smooth_curve(sub_points, sub_smooth_points, t_step);
        bezier_points.insert(bezier_points.end(), sub_smooth_points.begin(), sub_smooth_points.end());
    }

private:
    static std::vector<std::vector<size_t>> combination_table_;     // 记录组合数的表，用动态规划进行扩展

    /// @brief 计算组合数
    /// @param n 下标n
    /// @param r 上表r
    /// @return 组合数结果
    static size_t C_n_r(const size_t n, const size_t r);
};


/// @brief B样条曲线。控制点个数为n+1，阶数为k，函数次数为k-1。
class BSplineCurve
{
public:
    BSplineCurve() = default;
    ~BSplineCurve() = default;

    /// @brief 使用控制点生成B样条曲线。
    /// @tparam PointType 支持OpenCV格式的整型和浮点型数据点
    /// @param control_points 输入的原始路径点
    /// @param bspline_points 输出的平滑后的稠密序列点
    /// @param k B样条曲线的阶数，生成k-1次函数。通常使用4阶，此时保证曲率平滑。
    /// @param u_step B样条曲线中u的步增长度，t∈[0, 1]，每一步生成一个输出点
    template <typename PointType>
    static void smooth_curve(const std::vector<cv::Point_<PointType>> & control_points, std::vector<cv::Point2d> & bspline_points, const size_t k = 4, const double u_step = 0.005)
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
        generate_knots(k, n);
        for (double u = 0.0; u <= 1.0; u += u_step)
        {
            bik_u(u, k, n);
            cv::Point2d pt(0.0, 0.0);
            for (size_t i = 0; i <= n; i++)
            {
                const double & coff = b_matrix_[k - 1][i];
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
    static void generate_knots(const size_t k, const size_t n, const double min = 0.0, const double max = 1.0);

    /// @brief 用于生成在给定的u下，所有的Bik的计算值，储存在表中，每次调用都会重新生成。
    /// @param u 当前的u值。
    /// @param k 阶数，即生成k阶B样条曲线，k-1次函数
    /// @param n 控制点数-1
    static void bik_u(const double u, const size_t k, const size_t n);
};
