// piecewise_jerk_speed_smoother.cpp
#include "global_planning/smoothers/piecewise_jerk_speed_smoother.h"


namespace Smoother
{
// 1 求解变量：对于已知的n个(s, t)点，共3n个求解变量，每个点包含(s, s', s'')三个变量
// 2 目标函数：
//      ①加速度代价，a要小，即s''要小；
//      ②加加速度代价，j要小，即s'''要小；
//      ③速度偏离代价，即(s - s_ref)要小；
//      ④横向加速度代价，v_i * kappa_i最小，但考虑到这样不是二次型，因此改为v_i * kappa_ref_i最小。
// 3 约束条件：
//      ①连续性约束（物理约束），即s, v, a需要满足物理学的连续性约束；
//      ②加速度约束（车辆性能约束），即s''属于[s''_min, s''_max]；
//      ③速度约束（道路限速、曲率限速等），即s'属于[s'_lower, s'_upper]；
//      ④位置约束（障碍物限速），即s属于[s_lower, s_upper]；
//      ⑤起点约束，即s0, s'0, s''0都为给定值；
//      ⑥终点停车约束，即s_n - 1 < s_length，即最终s小于道路总长。
bool PiecewiseJerkSpeedSmoother::Solve(
    double s0, double v0, double a0,
    const std::vector<double> & s_lower,
    const std::vector<double> & s_upper,
    const std::vector<double> & v_lower,
    const std::vector<double> & v_upper,
    const std::vector<double> & a_lower,
    const std::vector<double> & a_upper,
    const std::vector<double> & v_ref,
    const std::vector<double> & kappa_ref,
    double s_end,
    std::vector<Path::TrajectoryPoint> & result) const
{
    // 0. 变量个数、约束个数
    const size_t N = s_lower.size();                          // 时间点数
    const size_t n_vars = 3 * N;                              // 变量总数
    const size_t n_eq = 2 * (N - 1);                          // 连续性等式约束
    const size_t n_bound = n_vars;                            // 变量界限不等式约束
    const size_t n_constraints = n_eq + n_bound;              // 总约束数

    // 1. 设置求解器配置
    OsqpEigen::Solver solver;
    solver.settings()->setVerbosity(false);
    solver.settings()->setWarmStart(true);
    solver.settings()->setTimeLimit(3);                       // 3秒超时
    solver.data()->setNumberOfVariables(n_vars);
    solver.data()->setNumberOfConstraints(n_constraints);

    // 2. 构建二次项矩阵 hessian (目标函数 0.5 * x^T H x + f^T x)，只填充上三角部分
    Eigen::SparseMatrix<double> hessian(n_vars, n_vars);
    // 预估非零元数量：加速度项 N 个，加加速度项 N 个对角线 + (N-1) 个交叉项，速度项 N 个，总共约 (4N-1) 个
    hessian.reserve(4 * N - 1);
    const double dt = dt_;
    const double dt_sq = dt * dt;
    const double inv_dt_sq = 1.0 / dt_sq;
    // 变量索引偏移
    const size_t idx_s = 0;
    const size_t idx_v = N;
    const size_t idx_a = 2 * N;
    // 2.1 加速度代价 (w_a * a_i^2) - 对角线
    for (size_t i = 0; i < N; ++i)
    {
        hessian.insert(idx_a + i, idx_a + i) = weights_.w_acceleration;
    }
    // 2.2 加加速度代价 (w_j * j_i^2) - 涉及 a_i 和 a_{i+1}
    const double coeff = weights_.w_jerk * inv_dt_sq;
    for (size_t i = 0; i < N - 1; ++i)
    {
        hessian.coeffRef(idx_a + i, idx_a + i)         += coeff;    // 注意使用 coeffRef 累加
        hessian.coeffRef(idx_a + i + 1, idx_a + i + 1) += coeff;
        hessian.insert(idx_a + i, idx_a + i + 1)       = -coeff;    // 上三角
    }
    // 2.3 速度偏差代价 (w_v * (v_i - v_ref_i)^2) 
    // 2.4 横向加速度代价(w_lat * (v_i * kappa_i) ^ 2)
    for (size_t i = 0; i < N; ++i)
    {
        const double kappa = kappa_ref[i];
        const double coeff = weights_.w_speed_deviation + weights_.w_lateral_acceleration * kappa * kappa;
        hessian.insert(idx_v + i, idx_v + i) = coeff;
    }
    hessian *= 2;
    if (!solver.data()->setHessianMatrix(hessian))
        return false;

    // 3.设置梯度向量 f/q
    // 3.1 速度偏差代价带来的梯度
    Eigen::VectorXd gradient = Eigen::VectorXd::Zero(n_vars);
    for (size_t i = 0; i < N; ++i)
    {
        gradient(idx_v + i) += -2.0 * weights_.w_speed_deviation * v_ref[i];
    }
    if (!solver.data()->setGradient(gradient))
        return false;

    // 4. 设置约束矩阵 A 和 上下线性边界 lb, ub
    Eigen::SparseMatrix<double> linear_matrix(n_constraints, n_vars);
    // 预估非零元：连续性约束每行约4-5个，界限约束每行1个。总非零元约 9 * (N-1) + n_vars
    linear_matrix.reserve(9 * (N - 1) + n_vars);
    Eigen::VectorXd lower_bound = Eigen::VectorXd::Zero(n_constraints);
    Eigen::VectorXd upper_bound = Eigen::VectorXd::Zero(n_constraints);
    // 4.1 连续性等式约束 (前 n_eq 行)
    for (size_t i = 0; i < N - 1; ++i)
    {
        // v 连续性: v_{i+1} - v_i - 0.5*dt*(a_i + a_{i+1}) = 0
        size_t row = 2 * i;
        linear_matrix.insert(row, idx_v     + i) = -1.0;
        linear_matrix.insert(row, idx_v + 1 + i) =  1.0;
        linear_matrix.insert(row, idx_a     + i) = -0.5 * dt;
        linear_matrix.insert(row, idx_a + 1 + i) = -0.5 * dt;
        lower_bound(row) = 0.0;
        upper_bound(row) = 0.0;

        // s 连续性: s_{i+1} - s_i - dt*v_i - (dt^2/3)*a_i - (dt^2/6)*a_{i+1} = 0
        row = 2 * i + 1;
        linear_matrix.insert(row, idx_s     + i) = -1.0;
        linear_matrix.insert(row, idx_s + 1 + i) =  1.0;
        linear_matrix.insert(row, idx_v     + i) = -dt;
        linear_matrix.insert(row, idx_a     + i) = -dt_sq / 3.0;
        linear_matrix.insert(row, idx_a + 1 + i) = -dt_sq / 6.0;
        lower_bound(row) = 0.0;
        upper_bound(row) = 0.0;
    }
    // 4.2 变量界限不等式约束 (后 n_bound 行，对应单位矩阵)
    // 4.2.1 设置通用变量约束
    for (size_t j = 0; j < n_vars; ++j)
    {
        size_t row = n_eq + j;
        linear_matrix.insert(row, j) = 1.0;
    }
    for (size_t i = 0; i < N; ++i)
    {
        // s 变量
        lower_bound(n_eq + idx_s + i) = s_lower[i];
        upper_bound(n_eq + idx_s + i) = s_upper[i];
        // v 变量
        lower_bound(n_eq + idx_v + i) = v_lower[i];
        upper_bound(n_eq + idx_v + i) = v_upper[i];
        // a 变量
        lower_bound(n_eq + idx_a + i) = a_lower[i];
        upper_bound(n_eq + idx_a + i) = a_upper[i];
    }
    // 4.2.2 起点硬约束：强制起点精确值
    lower_bound(n_eq + idx_s) = s0;
    upper_bound(n_eq + idx_s) = s0;
    lower_bound(n_eq + idx_v) = v0;
    upper_bound(n_eq + idx_v) = v0;
    lower_bound(n_eq + idx_a) = a0;
    upper_bound(n_eq + idx_a) = a0;
    // 4.2.3 终点停车约束：最后一个 s 点 <= s_end
    upper_bound(n_eq + idx_s + N - 1) = std::min(upper_bound(n_eq + idx_s + N - 1), s_end);
    if (!solver.data()->setLinearConstraintsMatrix(linear_matrix))
        return false;
    if (!solver.data()->setLowerBound(lower_bound))
        return false;
    if (!solver.data()->setUpperBound(upper_bound))
        return false;

    // 5. 求解
    if (!solver.initSolver())
        return false;
    if (solver.solveProblem() != OsqpEigen::ErrorExitFlag::NoError)
        return false;

    // 6. 保存数据
    const Eigen::VectorXd & solution = solver.getSolution();
    result.clear();
    result.reserve(N);
    for (size_t i = 0; i < N; ++i)
    {
        Path::TrajectoryPoint point;
        point.t = i * dt;
        point.s = solution(idx_s + i);
        point.v = solution(idx_v + i);
        point.a = solution(idx_a + i);
        // 其他字段（如 x, y, theta, kappa）将在后续轨迹生成中填充
        result.push_back(point);
    }
    return true;
}

} // namespace Smoother