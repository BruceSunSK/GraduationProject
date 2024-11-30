#include "global_planning/map/distance_map.h"


namespace Map
{
void DistanceMap::SetMap(const cv::Mat & cv_map)
{
    // 此处分辨率设为1，意义是在算法内部单位全部都是以栅格形式确定的，此处保持统一。
    // 此处地图原点为(0, 0)，意义是在算法内部已经统一成了opencv格式的坐标，只有在算法最终输出时再考虑实际地图分辨率、地图原点，因此此处地图原点为(0, 0)
    grid_map::GridMapCvConverter::initializeFromImage(cv_map, 1.0, map_, grid_map::Position::Zero());
    for (grid_map::GridMapIterator it(map_); !it.isPastEnd(); ++it)
    {
        const grid_map::Index grid_map_index = *it;
        const grid_map::Index image_index = it.getUnwrappedIndex();

        const float imageValue = cv_map.at<float>(image_index(0), image_index(1));
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
