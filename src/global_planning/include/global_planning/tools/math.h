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

/// @brief 计算某点处的朝向角
/// @tparam T 浮点型
/// @param dx 关于参数s的x坐标的导数
/// @param dy 关于参数s的y坐标的导数
/// @return 角度，弧度值
template<typename T>
T Heading(const T dx, const T dy)
{
    static_assert(std::is_floating_point<T>::value);
    return std::atan2(dy, dx);
}

/// @brief 计算某点处的曲率
/// @tparam T 浮点型
/// @param dx 关于参数s的x坐标的导数
/// @param dy 关于参数s的y坐标的导数
/// @param ddx 关于参数s的x坐标的二阶导数
/// @param ddy 关于参数s的y坐标的二阶导数
/// @return 曲率值，含正负号
template<typename T>
T Curvature(const T dx, const T dy, const T ddx, const T ddy)
{
    static_assert(std::is_floating_point<T>::value);
    return (dx * ddy - dy * ddx) / std::pow(dx * dx + dy * dy, 1.5);
}

/// @brief 计算某点处的曲率导数值
/// @tparam T 浮点型
/// @param dx 关于参数s的x坐标的导数
/// @param dy 关于参数s的y坐标的导数
/// @param ddx 关于参数s的x坐标的二阶导数
/// @param ddy 关于参数s的y坐标的二阶导数
/// @param dddx 关于参数s的x坐标的三阶导数
/// @param dddy 关于参数s的y坐标的三阶导数
/// @return 曲率导数值，含正负号
template<typename T>
T CurvatureRate(const T dx, const T dy, const T ddx, const T ddy, const T dddx, const T dddy)
{
    static_assert(std::is_floating_point<T>::value);
    const double a = dx * ddy - dy * ddx;
    const double b = dx * dddy - dy * dddx;
    const double c = dx * ddx + dy * ddy;
    const double d = dx * dx + dy * dy;
    return (b * d - 3.0 * a * c) / (d * d * d);
}

/// @brief 计算两点之间的直线上的所有点的栅格坐标，使用8领域扩展。
/// @param p1 直线的第一个点
/// @param p2 直线的第二个点
/// @param width 该直线的宽度。值为1时，仅有单行像素点；其余值时，按照该值为半宽（按照奇数处理）。
/// @return 两点之间的直线上的所有点
std::vector<cv::Point2i> Bresenham(const cv::Point2i & p1, const cv::Point2i & p2, const int width = 1);

} // namespace Math
