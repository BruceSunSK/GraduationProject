#pragma once
#include <vector>
#include <stdint.h>

#include <opencv2/core.hpp>


/// @brief 所有规划器的接口类
class GlobalPlannerInterface
{
public:
    /// @brief 所有规划器参数的接口类
    struct GlobalPlannerParams
    {
        virtual ~GlobalPlannerParams() = default;
    };

public:
    GlobalPlannerInterface() = default;
    virtual ~GlobalPlannerInterface() = default;

    virtual void initParams(const GlobalPlannerParams & params) = 0;                    // 初始化规划器参数
    virtual bool setMap(const cv::Mat & map) = 0;                                       // 设置栅格地图
    virtual void setMapInfo(const double res, const double ori_x, const double ori_y)   // 设置地图的参数信息
    {
        res_ = res;
        ori_x_ = ori_x;
        ori_y_ = ori_y;
        init_map_info_ = true;
    }
    virtual bool setStartPoint(const int x, const int y) = 0;           // 设置起点栅格坐标
    virtual bool setStartPoint(const cv::Point2i p) = 0;                // 设置起点栅格坐标
    virtual bool setEndPoint(const int x, const int y) = 0;             // 设置终点栅格坐标
    virtual bool setEndPoint(const cv::Point2i p) = 0;                  // 设置终点栅格坐标

    virtual bool getProcessedMap(cv::Mat & map) = 0;                    // 处理后的地图，即算法内部真正使用的地图
    virtual bool getRawPath(std::vector<cv::Point2i> & path) = 0;       // 原始的以栅格为单位的路径
    virtual bool getSmoothPath(std::vector<cv::Point2d> & path) = 0;    // 平滑优化后的离散路径，会补齐因栅格坐标系而偏移出的0.5个单位长度的偏差

protected:
    // 地图属性
    int rows_ = 0;
    int cols_ = 0;
    int channels_ = 0;
    double res_ = 0.0;
    double ori_x_ = 0.0;
    double ori_y_ = 0.0;

    // 设置参数
    bool init_map_ = false;
    bool init_map_info_ = false;
    bool init_start_node_ = false;
    bool init_end_node_ = false;
};
