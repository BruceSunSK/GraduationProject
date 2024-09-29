#include <ros/ros.h>

#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>

#include "global_planning/tools/path_simplification.h"


int main(int argc, char * argv[])
{
    ros::init(argc, argv, "test_path_simplification");
    ros::NodeHandle nh;

    std::vector<cv::Point2i> input = { {0, 50}, {10, 50}, {20, 60}, {30, 55}, {40, 45},
                                       {50, 50}, {60, 60}, {70, 40}, {80, 30}, {90, 35}, {100, 50} };
    std::vector<cv::Point2i> output;
    // PathSimplification::distance_threshold(input, output, 10);
    // PathSimplification::angle_threshold(input, output, 20);
    PathSimplification::Douglas_Peucker(input, output, 10);

    cv::Mat img = cv::Mat::zeros(101, 101, CV_8UC3);
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
