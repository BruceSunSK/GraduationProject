#include <ros/ros.h>

#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>

#include "global_planning/path/simplification.h"


int main(int argc, char * argv[])
{
    ros::init(argc, argv, "test_path_simplification");
    ros::NodeHandle nh;

    // 1. 测试三种基本路径简化算法
    // std::vector<cv::Point2i> input = { {0, 50}, {10, 50}, {20, 60}, {30, 55}, {40, 45},
    //                                    {50, 50}, {60, 60}, {70, 40}, {80, 30}, {90, 35}, {100, 50} };
    // std::vector<cv::Point2i> output;
    // PathSimplification::distance_threshold(input, output, 10);
    // PathSimplification::angle_threshold(input, output, 20);
    // PathSimplification::Douglas_Peucker(input, output, 10);

    // cv::Mat img = cv::Mat::zeros(101, 101, CV_8UC3);
    // for (size_t i = 1; i < input.size(); i++)
    // {
    //     cv::line(img, input[i], input[i - 1], { 0, 0, 255 }, 1);
    // }
    // for (size_t i = 1; i < output.size(); i++)
    // {
    //     cv::line(img, output[i], output[i - 1], { 100, 200, 255 }, 1);
    // }

    // 2. 测试Bresenham算法
    // std::vector<cv::Point2i> v1 = PathSimplification::Bresenham(cv::Point2i(10, 5),  cv::Point2i(80, 45), 1);
    // std::vector<cv::Point2i> v2 = PathSimplification::Bresenham(cv::Point2i(10, 10), cv::Point2i(80, 50), 2);
    // std::vector<cv::Point2i> v3 = PathSimplification::Bresenham(cv::Point2i(10, 20), cv::Point2i(80, 60), 3);
    // std::vector<cv::Point2i> v4 = PathSimplification::Bresenham(cv::Point2i(10, 30), cv::Point2i(80, 70), 4);
    // cv::Mat img = cv::Mat::zeros(101, 101, CV_8UC3);
    // for (size_t i = 0; i < v1.size(); i++)
    // {
    //     img.at<cv::Vec3b>(v1[i]) = cv::Vec3b(100, 100, 255);
    // }
    // for (size_t i = 0; i < v2.size(); i++)
    // {
    //     img.at<cv::Vec3b>(v2[i]) = cv::Vec3b(100, 100, 255);
    // }
    // for (size_t i = 0; i < v3.size(); i++)
    // {
    //     img.at<cv::Vec3b>(v3[i]) = cv::Vec3b(100, 100, 255);
    // }
    // for (size_t i = 0; i < v4.size(); i++)
    // {
    //     img.at<cv::Vec3b>(v4[i]) = cv::Vec3b(100, 100, 255);
    // }

    // 3. 测试改进的Douglas-Peucker算法
    cv::Mat img = cv::Mat::zeros(101, 101, CV_8UC3);
    cv::Mat obs = cv::Mat::zeros(101, 101, CV_8UC1);
    cv::line(img, cv::Point2i(30, 50), cv::Point2i(100, 50), cv::Scalar_<uint8_t>(200, 200, 200), 1);
    cv::line(obs, cv::Point2i(30, 50), cv::Point2i(100, 50), cv::Scalar_<uint8_t>(100), 1);

    std::vector<cv::Point2i> input = { {10, 50}, {20, 60}, {30, 55}, {40, 45},
                                       {50, 50}, {60, 60}, {70, 40}, {80, 30}, {90, 35} };
    std::vector<cv::Point2i> output;
    Path::Simplification::DPPlus(obs, input, output, 50, 10, 3);
    for (size_t i = 1; i < input.size(); i++)
    {
        cv::line(img, input[i], input[i - 1], { 0, 0, 255 }, 1);
    }
    for (size_t i = 1; i < output.size(); i++)
    {
        cv::line(img, output[i], output[i - 1], { 100, 200, 255 }, 1);
    }

    cv::namedWindow("Path Simplification", cv::WindowFlags::WINDOW_NORMAL | cv::WindowFlags::WINDOW_KEEPRATIO);
    cv::imshow("Path Simplification", img);
    cv::waitKey();

    return 0;
}
