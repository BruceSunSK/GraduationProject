#pragma once
#include <vector>
#include <stdint.h>

#include <opencv2/core.hpp>

class GlobalPlannerInterface
{
public:
    GlobalPlannerInterface() {}
    virtual ~GlobalPlannerInterface() {}

    virtual bool setMap(const cv::Mat & map) = 0;
    virtual bool setStartPoint(const int x, const int y) = 0;
    virtual bool setStartPoint(const cv::Point2i p) = 0;
    virtual bool setEndPoint(const int x, const int y) = 0;
    virtual bool setEndPoint(const cv::Point2i p) = 0;

    virtual bool getRawPath(std::vector<cv::Point2i> & path) = 0;       // 原始的以栅格为单位的路径
    virtual bool getSmoothPath(std::vector<cv::Point2f> & path) = 0;    // 平滑优化后的路径，但是仍然只是对栅格点进行操作，也就意味着xy方向有0.5个单位的偏差

    int rows() { return rows_; }
    int cols() { return cols_; }
    int channels() { return channels_; }

protected:
    int rows_ = 0;
    int cols_ = 0;
    int channels_ = 0;
};
