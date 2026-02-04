#pragma once
#include <cmath>

#include "global_planning/path/data_type.h"


namespace Path
{
namespace Utils
{

/// @brief 将Cartesian坐标系的xy值转换到Frenet坐标系的sl值
/// @param xy Cartesian坐标系的xy值
/// @param ref_xy xy点对应的参考线上的sl点的xy值
/// @param ref_s xy点对应参考线上的sl点的s值
/// @param ref_theta xy点对应参考线上的sl点的theta值
/// @return Frenet坐标系的sl值
PointSL XYtoSL(const PointXY xy, const PointXY ref_xy, const double ref_s, const double ref_theta);

/// @brief 将Cartesian坐标系的xy值转换到Frenet坐标系的s, l, l', l"值
/// @param xy Cartesian坐标系的xy值
/// @param theta 待转换点的朝向角theta
/// @param kappa 待转换点的曲率kappa
/// @param ref_xy xy点对应的参考线上的sl点的xy值
/// @param ref_s xy点对应参考线上的sl点的s值
/// @param ref_theta xy点对应参考线上的sl点的朝向角theta值
/// @param ref_kappa xy点对应参考线上的sl点的曲率kappa值
/// @param ref_kappa_prime xy点对应参考线上的sl点的曲率导数kappa'值
/// @return Frenet坐标系的s，l，l', l"值
PointSLWithDerivatives XYtoSL(const PointXY xy, const double theta, const double kappa,
                              const PointXY ref_xy, const double ref_s, const double ref_theta,
                              const double ref_kappa, const double ref_kappa_prime);

/// @brief 将Frenet坐标系的sl值转换到Cartesian坐标系的xy值
/// @param sl Frenet坐标系的sl值
/// @param ref_xy sl点对应的参考线上的xy点的xy值
/// @param ref_theta sl点对应的参考线上的xy点的theta值
/// @return Cartesian坐标系的xy值
PointXY SLtoXY(const PointSL sl, const PointXY ref_xy, const double ref_theta);

/// @brief 输入的reference和target是以global为原点的，现在将target转化为以reference为原点的坐标系下。不会转换sl值。
/// @param reference 转换后的原点
/// @param target 待转换点
/// @return 在reference坐标系下的target点
PathNode GlobalToLocal(const PathNode & reference, const PathNode & target);

} // namespace Utils
} // namespace Path
