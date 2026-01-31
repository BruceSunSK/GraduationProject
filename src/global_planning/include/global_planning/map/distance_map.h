#pragma once
#include <opencv2/core.hpp>

#include <grid_map_core/grid_map_core.hpp>
#include <grid_map_cv/grid_map_cv.hpp>


namespace Map
{
class DistanceMap
{
public:
    using Ptr = std::shared_ptr<DistanceMap>;
    
public:
    DistanceMap() = default;
    DistanceMap(const cv::Mat & cv_map) { SetMap(cv_map); }
    DistanceMap(const DistanceMap & grid_map) = delete;
    DistanceMap(DistanceMap && grid_map) = delete;
    DistanceMap & operator=(const DistanceMap & grid_map) = delete;
    DistanceMap & operator=(DistanceMap && grid_map) = delete;
    ~DistanceMap() = default;

    /// @brief 设置地图
    /// @param cv_map 图像地图
    void SetMap(const cv::Mat & cv_map);
    /// @brief 判断点是否在地图内
    /// @param x 点的 x 坐标
    /// @param y 点的 y 坐标
    /// @return 是否在地图内
    bool IsInside(const double x, const double y) const;
    /// @brief 判断点是否在地图内
    /// @param position 点的坐标
    /// @return 是否在地图内
    bool IsInside(const cv::Point2d & position) const
    {
        return IsInside(position.x, position.y);
    }
    /// @brief 获取距离
    /// @param x 点的 x 坐标
    /// @param y 点的 y 坐标
    /// @return 距离
    double GetDistance(const double x, const double y) const;
    /// @brief 获取距离
    /// @param position 点的坐标
    /// @return 距离
    double GetDistance(const cv::Point2d & position) const
    {
        return GetDistance(position.x, position.y);
    }
  
private:
    grid_map::GridMap map_;
};

struct MultiMap
{
    MultiMap() = default;
    MultiMap(int rows, int cols, double resolution, double origin_x, double origin_y, const cv::Mat & cost_map, const cv::Mat & distance_map):
        rows(rows), cols(cols), resolution(resolution), origin_x(origin_x), origin_y(origin_y),
        cost_map(cost_map), distance_map(distance_map){}

    int rows;
    int cols;
    double resolution;
    double origin_x;
    double origin_y;
    cv::Mat cost_map;
    DistanceMap distance_map;
};
} // namespace Map
