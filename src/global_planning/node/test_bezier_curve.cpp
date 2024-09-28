#include <ros/ros.h>

#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>

#include "global_planning/bezier_curve.h"


int main(int argc, char * argv[])
{
    ros::init(argc, argv, "test_bezier_curve");
    ros::NodeHandle nh;

    std::vector<cv::Point2d> input = {{0, 0}, {50, 0}, {50, 100}, {100, 100}};
    std::vector<cv::Point2d> output;
    BezierCurve::smooth_curve(input, output);

    cv::Mat img = cv::Mat::zeros(101, 101, CV_8UC3);
    img.at<cv::Vec3b>(0, 0) = {0, 0, 255};
    img.at<cv::Vec3b>(0, 50) = {0, 0, 255};
    img.at<cv::Vec3b>(100, 50) = {0, 0, 255};
    img.at<cv::Vec3b>(100, 100) = {0, 0, 255};
    for (auto &&p : output)
    {
        img.at<cv::Vec3b>(p.y + 0.5, p.x + 0.5) = {100, 200, 255};
    }
    

    cv::namedWindow("Bezier Curve", cv::WindowFlags::WINDOW_NORMAL | cv::WindowFlags::WINDOW_KEEPRATIO);
    cv::imshow("Bezier Curve", img);
    cv::waitKey();

    return 0;
}
