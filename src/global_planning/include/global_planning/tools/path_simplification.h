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
    /// @param threshold 垂距限值法中的垂距阈值，超过该阈值的点将保留，否则将剔除。
    template <typename PointType>
    static void distance_threshold(const std::vector<cv::Point_<PointType>> & in_path, std::vector<cv::Point_<PointType>> & out_path, double threshold = 1.0)
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

    /// @brief 使用角度限值法对路径进行抽稀
    /// @tparam PointType 支持OpenCV格式的整型和浮点型数据点
    /// @param in_path 待抽稀的原始路径
    /// @param out_path 抽稀后的路径
    /// @param threshold 角度限值法中的光栏开口尺寸，超过该光栏的点将保留，否则将剔除
    template <typename PointType>
    static void angle_threshold(const std::vector<cv::Point_<PointType>> & in_path, std::vector<cv::Point_<PointType>> & out_path, double threshold = 1.0)
    {
        using Point = cv::Point_<PointType>;

        out_path.clear();
        const size_t n = in_path.size();
        if (n <= 2)
        {
            out_path.insert(out_path.end(), in_path.begin(), in_path.end());
            return;
        }

        // 必定保留起点和终点
        out_path.push_back(in_path.front());

        // 计算初始光栏边界
        const Point * p1 = &in_path[0];
        const Point * p2 = &in_path[1];
        double al = distance_to_line_angle(*p1, *p2, threshold / 2);
        double angle_limit[2] = { -al, al };
        for (size_t r = 2; r < n; r++)
        {
            const Point * p3 = &in_path[r];

            // 计算当前角度
            const double angle = point_to_line_angle(*p3, *p1, *p2);
            // 判断是否在光栏内部
            if (angle > angle_limit[0] && angle < angle_limit[1])
            {
                p2 = p3;

                // 重新计算两个边界
                al = distance_to_line_angle(*p1, *p2, threshold / 2);
                if (angle - al > angle_limit[0])    // 下边界未超出光栏
                {
                    angle_limit[0] = -al;
                }
                else                                // 下边界超出光栏
                {
                    angle_limit[0] -= angle;
                }
                if (angle + al < angle_limit[1])    // 上边界未超出光栏
                {
                    angle_limit[1] = al;
                }
                else                                // 上边界超出光栏
                {
                    angle_limit[1] -= angle;
                }
            }
            else
            {
                p1 = p2;
                p2 = p3;

                // 保留节点并更新光栅边界
                out_path.push_back(*p1);
                al = distance_to_line_angle(*p1, *p2, threshold / 2);
                angle_limit[0] = -al;
                angle_limit[1] = al;
            }
        }
        
        out_path.push_back(in_path.back());
    }

    /// @brief 使用Douglas-Peucker法对路径进行抽稀
    /// @tparam PointType 支持OpenCV格式的整型和浮点型数据点
    /// @param in_path 待抽稀的原始路径
    /// @param out_path 抽稀后的路径
    /// @param threshold Douglas-Peucker法中的阈值。递归过程中超过阈值的点将保留，否则将剔除
    template <typename PointType>
    static void Douglas_Peucker(const std::vector<cv::Point_<PointType>> & in_path, std::vector<cv::Point_<PointType>> & out_path, double threshold = 1.0)
    {
        using Point = cv::Point_<PointType>;

        out_path.clear();
        const size_t n = in_path.size();
        if (n <= 2)
        {
            out_path.insert(out_path.end(), in_path.begin(), in_path.end());
            return;
        }

        // 递归实现
        std::function<void(const int, const int)> rdp =
            [&in_path, &out_path, threshold, &rdp](const int i_l, const int i_r) -> void
            {
                // 找距离最远点
                const Point base = in_path[i_r] - in_path[i_l];
                const double base_length = std::hypot<double>(base.x, base.y);
                int index = 0;
                double max = 0;
                for (size_t i = i_l + 1; i <= i_r - 1; i++)
                {
                    const Point v = in_path[i] - in_path[i_l];
                    const double dis = std::abs(v.cross(base)) / base_length;
                    if (dis > max)
                    {
                        max = dis;
                        index = i;
                    }
                }

                // 判断最远点距离是否超过阈值
                if (max > threshold)    // 超过就分别对该点左右两侧进行递归计算
                {
                    rdp(i_l, index);
                    out_path.pop_back();    // 左边多删除末尾点，以防重复
                    rdp(index, i_r);
                }
                else                    // 不超过就只保留起点和终点，其余都删除
                {
                    out_path.push_back(in_path[i_l]);
                    out_path.push_back(in_path[i_r]);
                }
            };

        rdp(0, n - 1);
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
        cv::Point_<PointType> v1 = p2 - p1;
        cv::Point_<PointType> v2 = p - p1;
        return std::abs(v1.cross(v2)) / std::hypot<double>(v2.x, v2.y);
    }

    /// @brief 计算点p到由p1p2构成的直线的角度。即直线p1p2和直线p1p的夹角角度
    /// @tparam PointType 支持OpenCV格式的整型和浮点型数据点
    /// @param p 待计算的点
    /// @param p1 直线的第一个点
    /// @param p2 直线的第二个点
    /// @return 点到直线的角度
    template <typename PointType>
    static double point_to_line_angle(const cv::Point_<PointType> & p, const cv::Point_<PointType> & p1, const cv::Point_<PointType> & p2)
    {
        cv::Point_<PointType> v1 = p2 - p1;
        cv::Point_<PointType> v2 = p - p1;
        return std::acos(v1.ddot(v2) / (std::hypot<double>(v1.x, v1.y) * std::hypot<double>(v2.x, v2.y)));
    }


    /// @brief p1p2构成的直线的在p2出垂直延伸出d后的点p，计算pp1和p1p2点夹角的大小。
    /// @tparam PointType 支持OpenCV格式的整型和浮点型数据点
    /// @param p1 直线的第一个点
    /// @param p2 直线的第二个点
    /// @return pp1和p1p2夹角的大小
    template <typename PointType>
    static double distance_to_line_angle(const cv::Point_<PointType> & p1, const cv::Point_<PointType> & p2, const double d)
    {
        return std::abs(std::atan2(d, std::hypot<double>(p2.x - p1.x, p2.y - p1.y)));
    }
};
