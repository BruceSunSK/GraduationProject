#pragma once
#include <array>
#include <tuple>
#include <memory>

#include <Eigen/Core>
#include <Eigen/Dense>

#include "global_planning/tools/math.h"

#include <iostream>
namespace Curve
{
template<size_t N>
class Polynomial
{
public:
    using Ptr = std::shared_ptr<Polynomial>;

public:
    Polynomial() = delete;
    /// @brief 构造N阶多项式，需要输入N+1项已知条件。
    /// @param conditions 输入N+1项已知条件。tuple格式为：<x, y_k, k>，分别对应x值，y的k阶导数值，y_k的k
    Polynomial(const std::array<std::tuple<double, double, size_t>, N + 1> & conditions)
    {
        static_assert(N > 0, "多项式阶数必须大于0");
        SetConditions(conditions);
    }
    Polynomial(const Polynomial & other) = delete;
    Polynomial(Polynomial && other) = delete;
    Polynomial & operator=(const Polynomial & other) = delete;
    Polynomial & operator=(Polynomial && other) = delete;
    ~Polynomial() = default;

    /// @brief 求多项式在给定x处的y_k值
    /// @param x 自变量x值
    /// @param derivative y的阶数
    /// @return x处的y阶导数的函数值
    double operator()(const double x, const size_t derivative = 0) const
    {
        if (derivative > N)
        {
            return 0.0;
        }

        double value = 0.0;
        double x_n = 1.0;
        for (size_t i = derivative; i <= N; i++)
        {
            size_t d_coeff = 1;
            size_t d_value = i;
            for (size_t m = 0; m < derivative; m++)
            {
                d_coeff *= d_value;
                --d_value;
            }

            value += (d_coeff * coefficients_(i) * x_n);
            x_n *= x;
        }
        return value;
    }

private:
    Eigen::VectorXd coefficients_;  // 多项式系数，从常数项a0开始储存
    
    void SetConditions(const std::array<std::tuple<double, double, size_t>, N + 1> & conditions)
    {
        // 构建矩阵A和向量b
        Eigen::MatrixXd A = Eigen::MatrixXd::Zero(N + 1, N + 1);
        Eigen::VectorXd b(N + 1);

        // 填充矩阵A和向量b
        for (size_t i = 0; i <= N; ++i)
        {
            const double & x   = std::get<0>(conditions[i]);
            const double & y_k = std::get<1>(conditions[i]);
            const size_t & k   = std::get<2>(conditions[i]);
            
            double x_n = 1.0;
            for (size_t j = k; j <= N; ++j)
            {
                size_t d_coeff = 1;
                size_t d_value = j;
                for (size_t m = 0; m < k; m++)
                {
                    d_coeff *= d_value;
                    --d_value;
                }
                
                A(i, j) = d_coeff * x_n;

                x_n *= x;
            }
            b(i) = y_k;
        }

        // 解线性方程组
        coefficients_ = A.colPivHouseholderQr().solve(b);
    }
};

} // namespace Curve
