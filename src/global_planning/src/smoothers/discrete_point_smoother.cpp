#include "global_planning/smoothers/discrete_point_smoother.h"


namespace Smoother
{
/// @brief 使用OSQP对二次规划问题进行求解。三种代价类型：平滑代价、长度代价（均匀代价）、偏离代价，对应的Hessian矩阵如下所示：
/// @details 下方是Apollo中Fem Smoother的注释，构造的Hessian一模一样。其中OSQP只需要输入Hessian矩阵的上三角即可。后续记得还需要乘2
/// Three quadratic penalties are involved:
/// 1. Penalty x on distance between middle point and point by finite element
/// estimate;
/// 2. Penalty y on path length;
/// 3. Penalty z on difference between points and reference points
/// 
/// General formulation of P matrix is as below(with 6 points as an example):
/// I is a two by two identity matrix, X, Y, Z represents x * I, y * I, z * I
/// 0 is a two by two zero matrix
/// |X+Y+Z, -2X-Y,   X,       0,       0,       0    |
/// |0,     5X+2Y+Z, -4X-Y,   X,       0,       0    |
/// |0,     0,       6X+2Y+Z, -4X-Y,   X,       0    |
/// |0,     0,       0,       6X+2Y+Z, -4X-Y,   X    |
/// |0,     0,       0,       0,       5X+2Y+Z, -2X-Y|
/// |0,     0,       0,       0,       0,       X+Y+Z|
bool DiscretePointSmoother::Solve(const Path::ReferencePath::Ptr & raw_ref_path, Path::ReferencePath::Ptr & ref_path)
{
    // 0. 变量个数、约束个数
    const size_t point_num = raw_ref_path->GetSize();
    const size_t variable_num = 2 * point_num;
    const size_t constraint_num = 2 * point_num;

    // 1. 设置求解器配置
    OsqpEigen::Solver solver;
    solver.settings()->setVerbosity(false);                 // 关闭输出
    solver.settings()->setWarmStart(true);                  // 开启warm start
    solver.settings()->setTimeLimit(3);                     // 3秒内求解完成，否则返回失败
    solver.data()->setNumberOfVariables(variable_num);      // 一个路点包含X、Y两个参数
    solver.data()->setNumberOfConstraints(constraint_num);  // 目前无约束，后续如果有约束的话，会对X、Y方向有大范围内的硬约束
    
    // 2. 设置Hessian矩阵 H/Q
    Eigen::SparseMatrix<double> hessian(variable_num, variable_num);
    const double X = weights_[0];
    const double Y = weights_[1];
    const double Z = weights_[2];
    // 2.1 第一列系数
    for (size_t i = 0; i < 2; ++i)
    {
        hessian.insert(i, i) = X + Y + Z;
    }
    // 2.2 第二列系数
    for (size_t i = 2; i < 4; ++i)
    {
        hessian.insert(i - 2, i) = -2 * X - Y;
        hessian.insert(i, i) = 5 * X + 2 * Y + Z;
    }
    // 2.3 倒数第一列系数
    for (size_t i = variable_num - 2; i < variable_num; ++i)
    {
        hessian.insert(i - 4, i) = X;
        hessian.insert(i - 2, i) = -2 * X - Y;
        hessian.insert(i, i) = X + Y + Z;
    }
    // 2.4 倒数第二列系数
    for (size_t i = variable_num - 4; i < variable_num - 2; ++i)
    {
        hessian.insert(i - 4, i) = X;
        hessian.insert(i - 2, i) = -4 * X - Y;
        hessian.insert(i, i) = 5 * X + 2 * Y + Z;
    }
    // 2.5 中间列系数
    for (size_t i = 4; i < variable_num - 4; ++i)
    {
        hessian.insert(i - 4, i) = X;
        hessian.insert(i - 2, i) = -4 * X - Y;
        hessian.insert(i, i) = 6 * X + 2 * Y + Z;
    }
    hessian *= 2;
    if (!solver.data()->setHessianMatrix(hessian))
        return false;

    // 3.设置梯度向量 f/q
    Eigen::VectorXd gradient = Eigen::VectorXd::Zero(variable_num);
    for (size_t i = 0; i < variable_num; i += 2)
    {
        gradient(i)     = -2.0 * Z * raw_ref_path->GetPathNodes()[i >> 1].x;
        gradient(i + 1) = -2.0 * Z * raw_ref_path->GetPathNodes()[i >> 1].y;
    }
    if (!solver.data()->setGradient(gradient))
        return false;

    // 4. 设置约束矩阵 A
    Eigen::SparseMatrix<double> linear_matrix(constraint_num, variable_num);
    // 目前只有对各个点的X、Y方向有硬约束，起点和终点完全与原始路径一致
    for (size_t i = 0; i < constraint_num; i++)
    {
        linear_matrix.insert(i, i) = 1.0;
    }
    if (!solver.data()->setLinearConstraintsMatrix(linear_matrix))
        return false;
    
    // 5. 设置上下线性边界 lb, ub
    Eigen::VectorXd lower_bound = Eigen::VectorXd::Zero(constraint_num);
    Eigen::VectorXd upper_bound = Eigen::VectorXd::Zero(constraint_num);
    // 5.1 起点和终点的硬约束
    lower_bound(0) = raw_ref_path->GetPathNodes().front().x;
    upper_bound(0) = raw_ref_path->GetPathNodes().front().x;
    lower_bound(1) = raw_ref_path->GetPathNodes().front().y;
    upper_bound(1) = raw_ref_path->GetPathNodes().front().y;
    lower_bound(constraint_num - 2) = raw_ref_path->GetPathNodes().back().x;
    upper_bound(constraint_num - 2) = raw_ref_path->GetPathNodes().back().x;
    lower_bound(constraint_num - 1) = raw_ref_path->GetPathNodes().back().y;
    upper_bound(constraint_num - 1) = raw_ref_path->GetPathNodes().back().y;
    // 5.2 范围约束
    for (size_t i = 2; i < constraint_num - 2; i += 2)
    {
        lower_bound(i)     = raw_ref_path->GetPathNodes()[i >> 1].x - buffer_;
        upper_bound(i)     = raw_ref_path->GetPathNodes()[i >> 1].x + buffer_;
        lower_bound(i + 1) = raw_ref_path->GetPathNodes()[i >> 1].y - buffer_;
        upper_bound(i + 1) = raw_ref_path->GetPathNodes()[i >> 1].y + buffer_;
    }
    if (!solver.data()->setLowerBound(lower_bound))
        return false;
    if (!solver.data()->setUpperBound(upper_bound))
        return false;

    // 6. 求解
    if (!solver.initSolver())
        return false;
    if (solver.solveProblem() != OsqpEigen::ErrorExitFlag::NoError)
        return false;

    // 7. 保存数据
    const Eigen::VectorXd & solution = solver.getSolution();
    std::vector<cv::Point2d> result_points;
    for (size_t i = 0; i < variable_num; i += 2)
    {
        result_points.emplace_back(solution(i), solution(i + 1));
    }
    ref_path = std::make_shared<Path::ReferencePath>(result_points, raw_ref_path->GetSInterval());
    return true;
}


} // namespace Smoother
