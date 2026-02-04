#include "local_planning/vehicle/collision.h"

namespace Vehicle
{
bool CollisionCircle::CheckCollision() const 
{
    for (auto && p : points_)
    {
        Path::PointXY xy;
        xy.x = (xy.x - map_.origin_x) / map_.resolution;
        xy.y = (xy.y - map_.origin_y) / map_.resolution;
        if (!map_.distance_map.IsInside(p.x, p.y) ||
            map_.distance_map.GetDistance(p.x, p.y) < radius_)
        {
            return true;
        }
    }
    return false;
}

std::array<std::pair<double, double>, 3> CollisionCircle::GetCollisionBounds(
    const double bound_search_range,
    const double bound_search_large_resolution,
    const double bound_search_small_resolution
) const
{
    std::array<std::pair<double, double>, 3> bounds;
    for (int i = 0; i < points_.size(); i++)
    {
        const auto & point = points_[i];

        // 确定下边界， 粗+细
        double lower_bound = 0.0;
        while (lower_bound > -bound_search_range)
        {
            lower_bound -= bound_search_large_resolution;
            Path::PointSL sl(0.0, lower_bound);                         // 这里准确讲不是sl到xy（不存在参考线），只是套用了公式，不需要s
            Path::PointXY xy = Path::Utils::SLtoXY(sl, { point.x, point.y }, point.theta);
            xy.x = (xy.x - map_.origin_x) / map_.resolution;
            xy.y = (xy.y - map_.origin_y) / map_.resolution;

            if (!map_.distance_map.IsInside(xy.x, xy.y) ||              // 此处点已经不在地图内部
                map_.distance_map.GetDistance(xy.x, xy.y) < radius_)    // 或者此处点发生了碰撞，则认为上次结果即为边界
            {
                lower_bound += bound_search_large_resolution;
                break;
            }
        }
        while (lower_bound > -bound_search_range)
        {
            lower_bound -= bound_search_small_resolution;
            Path::PointSL sl(0.0, lower_bound);
            Path::PointXY xy = Path::Utils::SLtoXY(sl, { point.x, point.y }, point.theta);
            xy.x = (xy.x - map_.origin_x) / map_.resolution;
            xy.y = (xy.y - map_.origin_y) / map_.resolution;

            if (!map_.distance_map.IsInside(xy.x, xy.y) ||
                map_.distance_map.GetDistance(xy.x, xy.y) < radius_)
            {
                lower_bound += bound_search_small_resolution;
                break;
            }
        }

        // 确定上边界， 粗+细
        double upper_bound = 0.0;
        while (upper_bound < bound_search_range)
        {
            upper_bound += bound_search_large_resolution;
            Path::PointSL sl(0.0, upper_bound);
            Path::PointXY xy = Path::Utils::SLtoXY(sl, { point.x, point.y }, point.theta);
            xy.x = (xy.x - map_.origin_x) / map_.resolution;
            xy.y = (xy.y - map_.origin_y) / map_.resolution;

            if (!map_.distance_map.IsInside(xy.x, xy.y) ||                               // 此处点已经不在地图内部
                map_.distance_map.GetDistance(xy.x, xy.y) < radius_)   // 或者此处点发生了碰撞，则认为上次结果即为边界
            {
                upper_bound -= bound_search_large_resolution;
                break;
            }
        }
        while (upper_bound < bound_search_range)
        {
            upper_bound += bound_search_small_resolution;
            Path::PointSL sl(0.0, upper_bound);
            Path::PointXY xy = Path::Utils::SLtoXY(sl, { point.x, point.y }, point.theta);
            xy.x = (xy.x - map_.origin_x) / map_.resolution;
            xy.y = (xy.y - map_.origin_y) / map_.resolution;

            if (!map_.distance_map.IsInside(xy.x, xy.y) ||
                map_.distance_map.GetDistance(xy.x, xy.y) < radius_)
            {
                upper_bound -= bound_search_small_resolution;
                break;
            }
        }

        bounds[i] = { lower_bound, upper_bound };
    }
    
    return bounds;
}

} // namespace Vehicle
