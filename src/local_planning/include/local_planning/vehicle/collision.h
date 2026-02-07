#pragma once
#include <vector>

#include "global_planning/map/distance_map.h"
#include "global_planning/path/data_type.h"
#include "global_planning/path/utils.h"
#include "global_planning/path/reference_path.h"


namespace Vehicle
{
// 本类的作用是用三碰撞圆覆盖车辆，然后用于碰撞检测和获得碰撞上下边界
// 输入：地图，三圆圆心点集，圆半径，安全距离
// 值得注意的是，判断碰撞时对于全部点进行检测，即只要有一个点碰撞，就认为该碰撞圆组发生碰撞；
class CollisionCircle
{
public:
    CollisionCircle() = delete;
    CollisionCircle(const Map::MultiMap & map, const std::array<Path::PathNode, 3> & points,
        const double radius, const double safety_distance = 0.1)
        : map_(map), points_(points), radius_(radius + safety_distance) {}
    CollisionCircle(const CollisionCircle & other) = delete;
    CollisionCircle(CollisionCircle && other) = delete;
    CollisionCircle & operator=(const CollisionCircle & other) = delete;
    CollisionCircle & operator=(CollisionCircle && other) = delete;
    ~CollisionCircle() = default;

    // 检测当前碰撞圆组内所有的圆，只要有一个圆碰撞，就认为发生碰撞。圆心点进需要xy点
    bool CheckCollision() const;
    // 将当前碰撞圆组内所有的圆的圆心，按照车辆横向方向向外探索距离边界
    std::array<std::pair<double, double>, 3> GetCollisionBounds(
        const double bound_search_range,
        const double bound_search_large_resolution, 
        const double bound_search_small_resolution,
        std::array<Path::PointXY, 3> * const lb_xy = nullptr,
        std::array<Path::PointXY, 3> * const ub_xy = nullptr) const;

private:
    const Map::MultiMap & map_;
    const std::array<Path::PathNode, 3> points_;
    const double radius_;
};

} // namespace Vehicle
