#pragma once
#include <vector>
#include <memory>

#include <opencv2/core.hpp>

#include <Eigen/Core>
#include <Eigen/Dense>
#include <Eigen/Sparse>


namespace Curve
{
class CubicSplineCurve
{
public:
    using Ptr = std::shared_ptr<CubicSplineCurve>;
    
    enum class BoundaryCondition
    {
        First_Derive,
        Second_Derive
    };

public:
    CubicSplineCurve() : left_bc_(BoundaryCondition::Second_Derive), right_bc_(BoundaryCondition::Second_Derive), left_value_(0.0), right_value_(0.0) {}
    /// @brief 根据边界条件，利用给定的点集生成三次样条曲线。调用后直接生成曲线结果，后续使用时直接调用operator()即可。
    /// @param points 给定的点集，需要保证x值递增
    /// @param left_bc 左边界条件
    /// @param left_value 左边界值
    /// @param right_bc 右边界条件
    /// @param right_value 右边界值
    CubicSplineCurve(const std::vector<cv::Point2d> & points,
                     const BoundaryCondition left_bc = BoundaryCondition::Second_Derive, const double left_value = 0.0,
                     const BoundaryCondition right_bc = BoundaryCondition::Second_Derive, const double right_value = 0.0)
                     : points_(points), left_bc_(left_bc), right_bc_(right_bc), left_value_(left_value), right_value_(right_value)
    {
        CalculateCoefficients();
    }
    CubicSplineCurve(std::vector<cv::Point2d> && points,
                     const BoundaryCondition left_bc = BoundaryCondition::Second_Derive, const double left_value = 0.0,
                     const BoundaryCondition right_bc = BoundaryCondition::Second_Derive, const double right_value = 0.0)
                     : points_(std::move(points)), left_bc_(left_bc), right_bc_(right_bc), left_value_(left_value), right_value_(right_value)
    {
        CalculateCoefficients();
    }
    CubicSplineCurve(const CubicSplineCurve & other) = delete;
    CubicSplineCurve(CubicSplineCurve && other) = delete;
    CubicSplineCurve & operator=(const CubicSplineCurve & other) = delete;
    CubicSplineCurve & operator=(CubicSplineCurve && other) = delete;
    ~CubicSplineCurve() = default;


    /// @brief 重新指定边界条件和边界值，之后在SetPoints前调用。
    /// @param left_bc 左边界条件
    /// @param left_value 左边界值
    /// @param right_bc 右边界条件
    /// @param right_value 右边界值
    void SetBoundary(const BoundaryCondition left_bc = BoundaryCondition::Second_Derive, const double left_value = 0.0,
                     const BoundaryCondition right_bc = BoundaryCondition::Second_Derive, const double right_value = 0.0)
    {
        left_bc_ = left_bc;
        right_bc_ = right_bc;
        left_value_ = left_value;
        right_value_ = right_value;
    }
    /// @brief 重新指定点集，并根据现有的边界条件和边界值重新计算三次样条曲线的系数，之后调用operator()即可。
    /// @param points 给定的点集，需要保证x值递增
    void SetPoints(const std::vector<cv::Point2d> & points)
    {
        points_ = points;
        CalculateCoefficients();
    }
    /// @brief 重新指定点集，并根据现有的边界条件和边界值重新计算三次样条曲线的系数，之后调用operator()即可。
    /// @param points 给定的点集，需要保证x值递增
    void SetPoints(std::vector<cv::Point2d> && points)
    {
        points_ = std::move(points);
        CalculateCoefficients();
    }
    /// @brief 根据给定的x值，利用三次样条曲线的系数插值计算对应的y值、一阶导数、二阶导数、三阶导数。
    /// @param x 待计算位置的x值
    /// @param derivative 导数阶数，0表示插值，1表示一阶导数，2表示二阶导数，3表示三阶导数
    /// @return 插值结果
    double operator()(const double x, const int derivative = 0) const;

private:
    BoundaryCondition left_bc_;
    BoundaryCondition right_bc_;
    double left_value_;
    double right_value_;
    std::vector<cv::Point2d> points_;

    Eigen::MatrixXd coeffs_;    // 三次样条曲线的系数。对于n个点，有n-1组系数，然后加上左右外界2组系数，因此此处共有n+1组系数。


    /// @brief 根据现有的点和边界条件计算样条系数
    void CalculateCoefficients();
};

} // namespace Curve
