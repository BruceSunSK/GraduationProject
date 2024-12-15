#pragma once
#include <vector>
#include <array>
#include <cmath>
#include <limits>
#include <algorithm>


namespace Curve
{
class Dubins
{
public:
    enum class DubinsSegmentType : uint8_t
    {
        UNKNOWN,
        L,
        S,
        R
    };

    struct DubinsPath
    {
        double r;
        std::array<double, 3> l;
        std::array<DubinsSegmentType, 3> type;

        DubinsPath() : r(0.0), l({ 0.0, 0.0, 0.0 }), type({ DubinsSegmentType::UNKNOWN, DubinsSegmentType::UNKNOWN, DubinsSegmentType::UNKNOWN }) {}
        DubinsPath(const double t, const double p, const double q, const std::array<DubinsSegmentType, 3> type, double r)
                  : l({ t, p, q }), type { type[0], type[1], type[2] }, r(r) {}

        /// @brief 归一化后的路径长度
        /// @return 路径长度
        double Length() const
        {
            return l[0] + l[1] + l[2];
        }

        /// @brief 真实路径长度
        /// @return 路径长度 
        double RealLength() const
        {
            return r * Length();
        }
    };
      
public:
    Dubins() = delete;
    Dubins(const double turning_radius, const double step_size = 0.1) : r_(turning_radius), step_(step_size) {}
    Dubins(const Dubins &) = delete;
    Dubins(Dubins &&) = delete;
    Dubins & operator=(const Dubins &) = delete;
    Dubins & operator=(Dubins &&) = delete;
    ~Dubins() = default;

    /// @brief 计算Dubins曲线路径
    /// @param start 起点x, y, yaw。单位m m rad
    /// @param end 终点x, y, yaw。单位m m rad
    /// @return dubins路径
    DubinsPath Path(const std::array<double, 3> & start, const std::array<double, 3> & end);

    /// @brief 将现有的dubins路径进行离散化，得到离散点
    /// @param path dubins路径
    /// @param start 与给定dubins路径匹配的起点x, y, yaw。单位m m rad
    /// @return 离散后的点集，x, y, yaw。单位m m rad
    std::vector<std::array<double, 3>> SegmentPath(const DubinsPath & path, const std::array<double, 3> & start) const;
    
private:
    double Mod2Pi(const double angle) const 
    {
        double a = angle;
        while (a < 0)
        {
            a+= 2.0 * M_PI;
        }
        if (2 * M_PI - a < 0.5 * EPS)
        {
            a = 0.0;
        }
        return a;
    }

    DubinsPath LSL() const;
    DubinsPath RSR() const;
    DubinsPath LSR() const;
    DubinsPath RSL() const;
    DubinsPath RLR() const;
    DubinsPath LRL() const;

    /// @brief 离散化路径，得到归一化后的长度s处的点
    /// @param path 待离散化的Dubins路径
    /// @param start 原始起点位置
    /// @param s 归一化后的长度s
    /// @return 离散点，x, y, yaw
    std::array<double, 3> Interpolate(const DubinsPath & path, const std::array<double, 3> & start, const double s) const;

    const double EPS = 1e-6;
    const double INF = std::numeric_limits<double>::infinity();
    const double ZERO = -1e-7;
    
    double r_;
    double step_;

    double d_;
    double theta_;
    double alpha_;
    double beta_;
    double ca_;
    double sa_;
    double cb_;
    double sb_;

};
} // namespace Curve
