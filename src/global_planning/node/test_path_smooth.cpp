#include <iostream>
#include <chrono>

#include <ros/ros.h>

#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>

#include "global_planning/tools/path_smooth.h"


int main(int argc, char * argv[])
{
    ros::init(argc, argv, "test_path_smooth");
    ros::NodeHandle nh;

    std::vector<cv::Point2d> input = { {0, 0}, {10, 0}, {20, 5}, { 50, 10 }, {50, 90}, {80, 95}, {90, 100}, { 100, 100 } };
    std::vector<cv::Point2d> output;
    cv::Mat img = cv::Mat::zeros(101, 101, CV_8UC3);
    for (auto && p : input)
    {
        img.at<cv::Vec3b>(p.y, p.x) = { 0, 0, 255 };
    }

    // 1. 使用Bezier曲线平滑路径
    auto t1 = std::chrono::steady_clock::now();
    BezierCurve::smooth_curve(input, output);
    std::cout << "Bezier time: " << (std::chrono::steady_clock::now() - t1).count() / 1e6 << " ms" << std::endl;
    for (auto && p : output)
    {
        img.at<cv::Vec3b>(p.y + 0.5, p.x + 0.5) = { 255, 100, 100 };    // 蓝色
    }

    // 2. 使用B样条曲线平滑路径
    auto t2 = std::chrono::steady_clock::now();
    BSplineCurve::smooth_curve(input, output);
    std::cout << "B-spline time: " << (std::chrono::steady_clock::now() - t2).count() / 1e6 << " ms" << std::endl;
    for (auto && p : output)
    {
        img.at<cv::Vec3b>(p.y + 0.5, p.x + 0.5) = { 100, 200, 255 };    // 黄色
    }

    cv::namedWindow("Smooth Curve", cv::WindowFlags::WINDOW_NORMAL | cv::WindowFlags::WINDOW_KEEPRATIO);
    cv::imshow("Smooth Curve", img);
    cv::waitKey();

    return 0;
}
