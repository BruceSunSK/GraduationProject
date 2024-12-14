#include <iostream>
#include <chrono>

#include <ros/ros.h>

#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>

#include "global_planning/curve/bezier.h"
#include "global_planning/curve/bspline.h"
#include "global_planning/curve/cubic_spline.h"
#include "global_planning/curve/polynomial.h"
#include "global_planning/curve/dubins.h"


int main(int argc, char * argv[])
{
    ros::init(argc, argv, "test_path_smooth");
    ros::NodeHandle nh;

    // std::vector<cv::Point2d> input = { {0, 0}, {10, 0}, {20, 5}, { 50, 10 }, {50, 90}, {80, 95}, {90, 100}, { 100, 100 } };
    std::vector<cv::Point2d> input = { {20, 50}, {25, 10}, {35, 5}, { 50, 10 }, {50, 90}, {65, 95}, {75, 90}, { 80, 50 } };
    std::vector<cv::Point2d> output;
    cv::Mat img = cv::Mat::zeros(101, 101, CV_8UC3);

    // 1. 使用Bezier曲线平滑路径
    // auto t1 = std::chrono::steady_clock::now();
    // Curve::Bezier::SmoothCurve(input, output, 0.005);
    // std::cout << "Bezier fit time: " << (std::chrono::steady_clock::now() - t1).count() / 1e6 << " ms" << std::endl;
    // for (auto && p : output)
    // {
    //     img.at<cv::Vec3b>(p.y + 0.5, p.x + 0.5) = { 255, 100, 100 };    // 蓝色
    // }

    // 2. 使用B样条曲线平滑路径
    // auto t2 = std::chrono::steady_clock::now();
    // Curve::BSpline::SmoothCurve(input, output, 4, 0.0025);
    // std::cout << "B-spline fit time: " << (std::chrono::steady_clock::now() - t2).count() / 1e6 << " ms" << std::endl;
    // for (auto && p : output)
    // {
    //     img.at<cv::Vec3b>(p.y + 0.5, p.x + 0.5) = { 100, 200, 255 };    // 黄色
    // }

    // 3. 使用三次样条曲线插值路径
    // auto t3 = std::chrono::steady_clock::now();
    // std::vector<cv::Point2d> X_S;
    // std::vector<cv::Point2d> Y_S;
    // double s = 0.0;
    // X_S.emplace_back(s, input.front().x);
    // Y_S.emplace_back(s, input.front().y);
    // for (size_t i = 1; i < input.size(); i++)
    // {
    //     s += cv::norm(input[i] - input[i - 1]);
    //     X_S.emplace_back(s, input[i].x);
    //     Y_S.emplace_back(s, input[i].y);
    // }
    // Curve::CubicSpline X_S_curve(std::move(X_S), Curve::CubicSpline::BoundaryCondition::Second_Derive, 0.0,
    //                                                   Curve::CubicSpline::BoundaryCondition::Second_Derive, 0.0);
    // Curve::CubicSpline Y_S_curve(std::move(Y_S), Curve::CubicSpline::BoundaryCondition::Second_Derive, 0.0,
    //                                                   Curve::CubicSpline::BoundaryCondition::Second_Derive, 0.0);
    // output.clear();
    // for (double s_temp = -20; s_temp < s + 20 ; s_temp += 0.5)
    // {
    //     output.emplace_back(X_S_curve(s_temp), Y_S_curve(s_temp));
    // }
    // std::cout << "Cubic Spline interpolate time: " << (std::chrono::steady_clock::now() - t3).count() / 1e6 << " ms" << std::endl;
    // for (auto && p : output)
    // {
    //     img.at<cv::Vec3b>(p.y + 0.5, p.x + 0.5) = { 147, 20, 255 };     // 粉色
    // }

    // 4. 绘制原始节点
    // for (auto && p : input)
    // {
    //     img.at<cv::Vec3b>(p.y, p.x) = { 255, 255, 255 };
    // }

    // 5. 测试多项式曲线
    // std::array<std::tuple<double, double, size_t>, 4> condtions;
    // condtions[0] = std::make_tuple(0, 1, 0);
    // condtions[1] = std::make_tuple(0, 0, 1);
    // condtions[2] = std::make_tuple(1, 6, 0);
    // condtions[3] = std::make_tuple(1, 13, 1);
    // Curve::Polynomial<3> poly(condtions);
    // std::cout << "x = 0.5, y = " << poly(0.5, 0) << std::endl;
    // std::cout << "x = 0.5, y' = " << poly(0.5, 1) << std::endl;
    // std::cout << "x = 2, y = " << poly(2, 0) << std::endl;
    // std::cout << "x = 2, y' = " << poly(2, 1) << std::endl;
    // std::cout << "x = 2, y\" = " << poly(2, 2) << std::endl;

    // 6. 测试dubins曲线
    auto t4 = std::chrono::steady_clock::now();
    cv::Point2d end(50, 50);
    double end_yaw = -90 * M_PI / 180;
    
    Curve::Dubins dubins(10, 0.5);
    for (size_t i = 0; i < 11; i++)
    {
        cv::Point2d start;
        start.x = 50 + 40 * std::cos(2 * M_PI * i / 11);
        start.y = 50 + 40 * std::sin(2 * M_PI * i / 11);
        double start_yaw = 2 * M_PI * i / 11;

        auto dubins_path = dubins.Path({ start.x, start.y, start_yaw }, { end.x, end.y, end_yaw });
        for (auto && p : dubins_path)
        {
            img.at<cv::Vec3b>(p[1] + 0.5, p[0] + 0.5) = cv::Vec3b( i / 11.0 * 255.0,
                                                                   i / 11.0 * 255.0,
                                                                   (1 - i / 11.0) * 255.0);
        }
    }
    img.at<cv::Vec3b>(end) = { 100, 100, 255 };
    std::cout << "dubins time: " << (std::chrono::steady_clock::now() - t4).count() / 1e6 << " ms" << std::endl;
        
    cv::namedWindow("Smooth Curve", cv::WindowFlags::WINDOW_NORMAL | cv::WindowFlags::WINDOW_KEEPRATIO);
    cv::imshow("Smooth Curve", img);
    cv::waitKey();

    return 0;
}
