#pragma once

#include "global_planning/global_planner_interface.h"
#include "global_planning/tools/print_struct_and_enum.h"

class RRT : public GlobalPlannerInterface
{
public:
    RRT() = default;
    ~RRT() = default;


    /// @brief 对输入的map的进行二值化处理，然后设置成为规划器中需要使用的地图
    /// @param map 输入的原始地图
    /// @return 地图设置是否成功
    bool setMap(const cv::Mat & map) override;

private:
    cv::Mat map_;               // 代价地图
    cv::Point2i start_point_;   // 起点，真实地图上的起点落在栅格地图中的坐标
    cv::Point2i end_point_;     // 终点，真实地图上的终点落在栅格地图中的坐标
};
