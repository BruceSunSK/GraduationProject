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
private:
    enum class DubinsSegmentType : uint8_t
    {
        UNKNOWN,
        L,
        S,
        R
    };

    struct DubinsPath
    {
        std::array<double, 3> l;
        std::array<DubinsSegmentType, 3> type;

        DubinsPath() : l({ 0.0, 0.0, 0.0 }), type({ DubinsSegmentType::UNKNOWN, DubinsSegmentType::UNKNOWN, DubinsSegmentType::UNKNOWN }) {}
        DubinsPath(const double t, const double p, const double q, const std::array<DubinsSegmentType, 3> type)
                  : l({ t, p, q }), type { type[0], type[1], type[2] } {}

        double length() const
        {
            return l[0] + l[1] + l[2];
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
    /// @param start 起点x, y, yaw
    /// @param end 终点x, y, yaw
    /// @return 路径点集，每个点为x, y, yaw, s
    std::vector<std::array<double, 4>> Path(const std::array<double, 3> & start, const std::array<double, 3> & end);

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
    /// @return 离散点，x, y, yaw, s
    std::array<double, 4> Interpolate(const DubinsPath & path, const std::array<double, 3> & start, const double s) const;

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
