#include "global_planning/curve/bezier.h"


namespace Curve
{
std::vector<std::vector<size_t>> Bezier::combination_table_ = { {1}, {1, 1}, {1, 2, 1}, {1, 3, 3, 1} };

size_t Bezier::C_n_r(const size_t n, const size_t r)
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

} // namespace Curve
