#pragma once
#include <vector>

#include <opencv2/core.hpp>


class PathSimplification
{
public:
    PathSimplification() = default;
    ~PathSimplification() = default;

    /// @brief 使用垂距限值法对路径进行抽稀
    /// @tparam PointType 支持OpenCV格式的整型和浮点型数据点
    /// @param in_path 待抽稀的原始路径
    /// @param out_path 抽稀后的路径
    /// @param threshold 垂距限值法中的垂距阈值，超过该阈值的点将保留，否则将剔除
    template <typename PointType>
    static void perpendicular_distance_threshold(const std::vector<cv::Point_<PointType>> & in_path, std::vector<cv::Point_<PointType>> & out_path, double threshold = 1.0)
    {
        out_path.clear();
        const size_t n = in_path.size();
        if (n <= 2)
        {
            out_path.insert(out_path.end(), in_path.begin(), in_path.end());
            return;
        }

        // 必定保留起点和终点
        out_path.push_back(in_path.front());
        for (size_t l = 0, i = 1, r = 2; r < n; i++, r++)
        {
            const double dis = point_to_line_distance(in_path[i], in_path[l], in_path[r]);
            if (dis > threshold)
            {
                out_path.push_back(in_path[i]);
                l = i;
            }
        }
        out_path.push_back(in_path.back());
    }

private:
    /// @brief 计算点p到直线p1p2的距离
    /// @tparam PointType 支持OpenCV格式的整型和浮点型数据点
    /// @param p 待计算的点
    /// @param p1 直线的第一个点
    /// @param p2 直线的第二个点
    /// @return 点到直线的距离
    template <typename PointType>
    static double point_to_line_distance(const cv::Point_<PointType> & p, const cv::Point_<PointType> & p1, const cv::Point_<PointType> & p2)
    {
        cv::Point_<PointType> d = p2 - p1;
        cv::Point_<PointType> v = p - p1;
        return std::abs(d.cross(v)) / std::hypot<double>(v.x, v.y);
    }
};
