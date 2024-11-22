#include "global_planning/tools/math.h"


namespace Math
{
std::vector<cv::Point2i> Bresenham(const cv::Point2i & p1, const cv::Point2i & p2, const int width)
{
    if (width <= 0)
        return std::vector<cv::Point2i>();

    int dx = std::abs(p1.x - p2.x);
    int dy = std::abs(p1.y - p2.y);
    int ux = p1.x < p2.x ? 1 : -1;
    int uy = p1.y < p2.y ? 1 : -1;
    int eps = 0;
    int x = p1.x;
    int y = p1.y;
    int w = width == 1 ? 0 : (width + 1) / 2;

    std::vector<cv::Point2i> line_points;
    if (dx > dy)
    {
        for (; x != p2.x; x += ux)
        {
            for (int i = -w; i <= w; i++)
            {
                line_points.push_back(cv::Point2i(x, y + i));
            }

            eps += dy;
            if (2 * eps >= dx)
            {
                y += uy;
                eps -= dx;
            }
        }
        for (int i = -w; i <= w; i++)
        {
            line_points.push_back(cv::Point2i(x, y + i));
        }
    }
    else
    {
        for (; y != p2.y; y += uy)
        {
            for (int i = -w; i <= w; i++)
            {
                line_points.push_back(cv::Point2i(x + i, y));
            }

            eps += dx;
            if (2 * eps >= dy)
            {
                x += ux;
                eps -= dy;
            }
        }
        for (int i = -w; i <= w; i++)
        {
            line_points.push_back(cv::Point2i(x + i, y));
        }
    }

    return line_points;
}

} // namespace Math
