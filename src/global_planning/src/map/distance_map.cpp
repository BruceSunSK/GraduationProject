#include "global_planning/map/distance_map.h"


namespace Map
{
void DistanceMap::SetMap(const cv::Mat & cv_map)
{
    // 1. 此处分辨率设为1，意义是在算法内部单位全部都是以栅格形式确定的，此处保持统一。
    // 2. 此处地图原点为图形中央，与grid_map自身的坐标系有关。
    // 3. 地图坐标系需要旋转。直接用grid_map::cv接口的话，地图x方向大小是cv的rows，y方向是大小时cv的cols，不符合需要。
    //    导致后续带入点还需要转换。因此在构建地图时对opencv的地图旋转，让grid_map与常规地图保持一致。
    const int rows = cv_map.rows;
    const int cols = cv_map.cols;
    map_.setGeometry({ cols, rows }, 1.0, { cols / 2.0, rows / 2.0 });
    map_.setFrameId("map");
    map_.add("distance");
    for (grid_map::GridMapIterator it(map_); !it.isPastEnd(); ++it)
    {
        // 分辨率为1.0，栅格肯定一致，不需要用unwarrpedIndex
        const grid_map::Index grid_map_index = *it;

        // 此处转换用于对齐坐标系，使得后续调用opencv栅格坐标时的(x, y)和此时地图中的(x, y)完全一致
        const float imageValue = cv_map.at<float>(rows - 1 - grid_map_index(1), cols - 1 - grid_map_index(0));
        map_["distance"](grid_map_index(0), grid_map_index(1)) = imageValue;
    }
}

bool DistanceMap::IsInside(const double x, const double y) const
{
    return map_.isInside(grid_map::Position(x, y));
}

double DistanceMap::GetDistance(const double x, const double y) const
{
    return map_.atPosition("distance", grid_map::Position(x, y), grid_map::InterpolationMethods::INTER_LINEAR);
}

} // namespace Map
