#pragma once
#include <vector>
#include <cmath>

#include <opencv2/core.hpp>


namespace Math
{
/// @brief 将角度规范化到(-pi, pi]的范围内
/// @tparam T 浮点型
/// @param angle 弧度值角度
/// @return 规范后的弧度角度
template<typename T>
T NormalizeAngle(T angle)
{
    static_assert(std::is_floating_point<T>::value);
    while (angle > M_PI)
        angle -= 2 * M_PI;
    while (angle <= -M_PI)
        angle += 2 * M_PI;
    return angle;
}

/// @brief 计算两点之间的直线上的所有点的栅格坐标，使用8领域扩展。
/// @param p1 直线的第一个点
/// @param p2 直线的第二个点
/// @param width 该直线的宽度。值为1时，仅有单行像素点；其余值时，按照该值为半宽（按照奇数处理）。
/// @return 两点之间的直线上的所有点
std::vector<cv::Point2i> Bresenham(const cv::Point2i & p1, const cv::Point2i & p2, const int width = 1);

} // namespace Math
