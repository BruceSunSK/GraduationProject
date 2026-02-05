#include "global_planning/smoothers/piecewise_jerk_smoother2.h"


namespace Smoother
{
/// @brief 用OSQP求解器求解，起点硬约束，终点软约束。基本和apollo一致，只是在连续性约束中取消了对l''的连续性约束；将起点约束合并到边界约束中
/// 代价函数由以下几部分组成：1.偏离代价(l要小)，2.平滑代价(l', l''，l'''要小)，3.终点代价(l(n - 1)要接近终点)
/// 约束条件：1.碰撞圆边界约束(包含起点约束)，2.连续性约束。
/// 公式参考：https://zhuanlan.zhihu.com/p/480298921
bool PiecewiseJerkSmoother2::Solve(const Path::ReferencePath::Ptr & ref_path, const std::vector<std::array<std::pair<double, double>, 3>> & bounds,
                                   const std::array<double, 3> & init_state, const std::array<double, 3> & end_state_ref, std::vector<cv::Point2d> & optimized_path) const
{
    // 0. 变量个数、约束个数
    const size_t point_num = bounds.size();
    const size_t variable_num = 3 * point_num;              // 一个路点包含l, l', l''三个参数（包含起点约束）
    const size_t constraint_num = 3 * point_num             // 3n个针对l的碰撞圆边界约束，
                                + 2 * point_num             // 2n个针对l', l''的边界约束，
                                + 2 * (point_num - 1);      // 2(n-1)个针对l, l'的连续性约束
    // 此处这样计算ds会导致每个参考路点和bound并不是完全对应的，会有误差，但因为bound本身就存在误差，此处认为这个误差可以忽略。
    const double ds = ref_path->GetLength() / (point_num - 1);

    // 1. 设置求解器配置
    OsqpEigen::Solver solver;
    solver.settings()->setVerbosity(false);                 // 关闭输出
    solver.settings()->setWarmStart(true);                  // 开启warm start
    solver.settings()->setTimeLimit(3);                     // 3秒内求解完成，否则返回失败
    solver.data()->setNumberOfVariables(variable_num);      
    solver.data()->setNumberOfConstraints(constraint_num);

    // 2. 设置Hessian矩阵 H/Q
    Eigen::SparseMatrix<double> hessian(variable_num, variable_num);
    // 2.1 l部分系数
    for (size_t i = 0; i < point_num - 1; ++i)
    {
        hessian.insert(i, i) = weight_l_;
    }
    hessian.insert(point_num - 1, point_num - 1) = weight_l_ + weight_end_state_[0];
    // 2.2 l'部分系数
    for (size_t i = point_num; i < 2 * point_num - 1; ++i)
    {
        hessian.insert(i, i) = weight_dl_;
    }
    hessian.insert(2 * point_num - 1, 2 * point_num - 1) = weight_dl_ + weight_end_state_[1];
    // 2.3 l''部分系数
    const double ds_square = ds * ds;
    const double ds_square_inv = 1.0 / ds_square;
    // 2.3.1 第一列对角线
    hessian.insert(2 * point_num, 2 * point_num) = weight_ddl_ + weight_dddl_ * ds_square_inv;
    // 2.3.2 中间列对角线
    for (size_t i = 2 * point_num + 1; i < 3 * point_num - 1; ++i)
    {
        hessian.insert(i, i) = weight_ddl_ + 2 * weight_dddl_ * ds_square_inv;
    }
    // 2.3.3 最后一列对角线
    hessian.insert(3 * point_num - 1, 3 * point_num - 1) = weight_ddl_ + weight_dddl_ * ds_square_inv + weight_end_state_[2];
    // 2.3.4 对角线上方元素
    for (size_t i = 2 * point_num; i < 3 * point_num - 1; ++i)
    {
        // apollo 在这里要*2？
        hessian.insert(i, i + 1) = -weight_dddl_ * ds_square_inv;
    }
    hessian *= 2;
    if (!solver.data()->setHessianMatrix(hessian))
        return false;

    // 3.设置梯度向量 f/q
    Eigen::VectorXd gradient = Eigen::VectorXd::Zero(variable_num);
    // 3.1 终点代价带来的梯度
    gradient(point_num - 1) += -2.0 * weight_end_state_[0] * end_state_ref[0];
    gradient(point_num - 1) += -2.0 * weight_end_state_[1] * end_state_ref[1];
    gradient(point_num - 1) += -2.0 * weight_end_state_[2] * end_state_ref[2];
    if (!solver.data()->setGradient(gradient))
        return false;

    // 4. 设置约束矩阵 A 和 上下线性边界 lb, ub
    Eigen::SparseMatrix<double> linear_matrix(constraint_num, variable_num);
    Eigen::VectorXd lower_bound = Eigen::VectorXd::Zero(constraint_num);
    Eigen::VectorXd upper_bound = Eigen::VectorXd::Zero(constraint_num);
    const size_t vari_l = 0;
    const size_t vari_dl = vari_l + point_num;
    const size_t vari_ddl = vari_dl + point_num;
    // 4.1 边界约束
    const size_t cons_bound_l = 0;                              // l边界约束在A矩阵中的起始行号
    const size_t cons_bound_dl = cons_bound_l + 3 * point_num;  // dl边界约束在A矩阵中的起始行号
    const size_t cons_bound_ddl = cons_bound_dl + point_num;    // ddl边界约束在A矩阵中的起始行号
    // 4.1.1 l项
    linear_matrix.insert(vari_l, vari_l) = 1.0;
    lower_bound(vari_l) = init_state[0];
    upper_bound(vari_l) = init_state[0];
    for (size_t i = vari_l + 1; i < vari_dl; ++i)
    {
        linear_matrix.insert(i, i) = 1.0;
        lower_bound(i) = bounds[i].first;
        upper_bound(i) = bounds[i].second;
    }
    // 4.1.2 l'项
    linear_matrix.insert(vari_dl, vari_dl) = 1.0;
    lower_bound(vari_dl) = init_state[1];
    upper_bound(vari_dl) = init_state[1];
    for (size_t i = vari_dl + 1; i < vari_ddl; ++i)
    {
        linear_matrix.insert(i, i) = 1.0;
        lower_bound(i) = -dl_limit_;
        upper_bound(i) = dl_limit_;
    }
    // 4.1.3 l''项
    linear_matrix.insert(vari_ddl, vari_ddl) = 1.0;
    lower_bound(vari_ddl) = init_state[2];
    upper_bound(vari_ddl) = init_state[2];
    for (size_t i = vari_ddl + 1; i < variable_num; ++i)
    {
        linear_matrix.insert(i, i) = 1.0;
        const Path::PathNode ref_node = ref_path->GetPathNode((i - 2 * point_num) * ds);
        lower_bound(i) = -vehicle_kappa_max_ - ref_node.kappa;
        upper_bound(i) = vehicle_kappa_max_ - ref_node.kappa;
    }
    // 4.2 连续性约束。理论上还有l"的连续性约束，需要传入l'''的上下边界，我偷懒直接省略掉这个了。
    const size_t cons_continuity_dl = cons_bound_ddl + point_num;           // dl连续性约束在A矩阵中的起始行号
    const size_t cons_continuity_l = cons_continuity_dl + point_num - 1;    // l连续性约束在A矩阵中的起始行号
    // 4.2.1 l'项
    for (size_t i = 0; i < point_num - 1; ++i)
    {
        linear_matrix.insert(cons_continuity_dl + i, vari_dl      + i) = -1.0;
        linear_matrix.insert(cons_continuity_dl + i, vari_dl  + 1 + i) =  1.0;
        linear_matrix.insert(cons_continuity_dl + i, vari_ddl     + i) = -0.5 * ds;
        linear_matrix.insert(cons_continuity_dl + i, vari_ddl + 1 + i) = -0.5 * ds;
        lower_bound(cons_continuity_dl + i) = 0.0;
        upper_bound(cons_continuity_dl + i) = 0.0;
    }
    // 4.2.2 l项
    for (size_t i = 0; i < point_num - 1; ++i)
    {
        linear_matrix.insert(cons_continuity_l + i, vari_l       + i) = -1.0;
        linear_matrix.insert(cons_continuity_l + i, vari_l   + 1 + i) =  1.0;
        linear_matrix.insert(cons_continuity_l + i, vari_dl      + i) = -ds;
        linear_matrix.insert(cons_continuity_l + i, vari_ddl     + i) = -ds_square / 3.0;
        linear_matrix.insert(cons_continuity_l + i, vari_ddl + 1 + i) = -ds_square / 6.0;
        lower_bound(cons_continuity_l + i) = 0.0;
        upper_bound(cons_continuity_l + i) = 0.0;
    }
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
    optimized_path.clear();
    for (size_t i = 0; i < point_num; ++i)
    {
        const Path::PathNode ref_node = ref_path->GetPathNode(i * ds);
        const Path::PointSL sl(ref_node.s, solution(i));
        const Path::PointXY xy = Path::Utils::SLtoXY(sl, { ref_node.x, ref_node.y }, ref_node.theta);
        optimized_path.emplace_back(xy.x, xy.y);
    }
    return true;
}


} // namespace Smoother
