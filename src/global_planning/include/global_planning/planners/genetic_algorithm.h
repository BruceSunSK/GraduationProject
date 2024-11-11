#pragma once

#include <random>

#include "global_planning/planners/global_planner_interface.h"
#include "global_planning/tools/print_struct_and_enum.h"
#include "global_planning/tools/path_simplification.h"


/// @brief 基于遗传算法(GA, Genetic Algorithm)的全局路径规划器。
/// 随机点生成和后续使用都是离散点而非栅格点，然后放在栅格地图中进行障碍物判断。
class GA : public GlobalPlannerInterface
{

};