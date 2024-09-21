#pragma once
#include <vector>

#include <opencv2/core.hpp>

class BezierCurve
{
public:
    BezierCurve() {}
    ~BezierCurve() = default;

    /// @brief 根据控制点进行贝塞尔平滑
    /// @tparam PointType 支持OpenCV格式的整型和浮点型数据点
    /// @param control_points 输入的原始控制点
    /// @param bezier_points 输出的平滑后的稠密序列点
    /// @param t_step 贝塞尔平滑中t的步增长度，t∈[0, 1]，每一步生成一个输出点
    template <typename PointType>
    void smooth_curve(const std::vector<PointType> & control_points, std::vector<cv::Point2d> & bezier_points, double t_step = 0.01)
    {
        if (control_points.size() == 0)
        {
            bezier_points.clear();
            return;
        }
        if (control_points.size() == 1)
        {
            bezier_points.clear();
            bezier_points.push_back(cv::Point2d(control_points[0].x, control_points[0].y));
            return;
        }
        
        bezier_points.clear();
        size_t n = control_points.size() - 1;
        for (double t = 0; t <= 1; t += t_step)
        {
            cv::Point2d pt(0.0f, 0.0f);
            for (size_t i = 0; i <= n; i++)
            {
                pt.x += (C_n_r(n, i) * std::pow(t, i) * std::pow(1 - t, n - i) * control_points[i].x);
                pt.y += (C_n_r(n, i) * std::pow(t, i) * std::pow(1 - t, n - i) * control_points[i].y);
            }
            bezier_points.push_back(pt);
        }
    }

private:
    static std::vector<std::vector<size_t>> combination_table_;

    size_t C_n_r(size_t n, size_t r)
    {
        size_t size = combination_table_.size();
        if (n < size)
        {
            return combination_table_[n][r];
        }
        else
        {
            for (size_t i = size; i <= n; i++)
            {
                std::vector<size_t> row(i + 1);
                row[0] = 1;
                row[i] = 1;
                for (size_t j = 1; j < i; j++)
                {
                    row[j] = combination_table_[i - 1][j - 1] + combination_table_[i - 1][j];
                }
                combination_table_.push_back(row);
            }
            return combination_table_[n][r];
        }
    }
};