#include "global_planning/tools/path_smooth.h"


// ================================ BezierCurve ================================
std::vector<std::vector<size_t>> BezierCurve::combination_table_ = { {1}, {1, 1}, {1, 2, 1}, {1, 3, 3, 1} };

size_t BezierCurve::C_n_r(const size_t n, const size_t r)
{
    const size_t size = combination_table_.size();
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
            combination_table_.push_back(std::move(row));
        }
        return combination_table_[n][r];
    }
}
// ================================ BezierCurve ================================


// ================================ BSplineCurve ================================
std::vector<double> BSplineCurve::knots_ = {};
std::vector<std::vector<double>> BSplineCurve::b_matrix_ = {};

void BSplineCurve::generate_knots(const size_t k, const size_t n, const double min, const double max)
{
    knots_.clear();
    knots_.resize(n + k + 1);
    const size_t piecewise = n - k + 2;
    std::fill(knots_.begin(), knots_.begin() + k, min);
    for (size_t i = k; i < n + 1; i++)
    {
        knots_[i] = knots_[i - 1] + 1.0 / piecewise;
    }
    std::fill(knots_.rbegin(), knots_.rbegin() + k, max);
}

void BSplineCurve::bik_u(const double u, const size_t k, const size_t n)
{
    b_matrix_.clear();
    const size_t k_1 = k - 1;

    // 第0行特殊处理
    std::vector<double> row_0(n + k, 0.0);
    for (size_t j = k; j < knots_.size(); j++)  // 寻找当前u所在的区间，由于knots_是递增的，所以只需要比较大小即可。且由于准样条，则从k开始比较即可。
    {
        if (u < knots_[j])
        {
            row_0[j - 1] = 1.0;
            break;
        }
    }
    b_matrix_.push_back(std::move(row_0));

    // 第i行的计算
    for (size_t i = 1; i <= k_1; i++)
    {
        // 第i行前/后k-i-1个元素，都为一定0
        std::vector<double> row_vec(n + k - i, 0.0);

        // 其余迭代计算
        for (size_t j = k - i - 1; j < n + 1; j++)
        {
            double de1 = knots_[i + j] - knots_[j];
            double de2 = knots_[i + j + 1] - knots_[j + 1];
            if (de1 == 0.0)
            {
                de1 = 1.0;
            }
            if (de2 == 0.0)
            {
                de2 = 1.0;
            }
            row_vec[j] = (u - knots_[j]) / de1 * b_matrix_[i - 1][j] +
                         (knots_[i + j + 1] - u) / de2 * b_matrix_[i - 1][j + 1];
        }
        b_matrix_.push_back(std::move(row_vec));
    }
}
// ================================ BSplineCurve ================================
