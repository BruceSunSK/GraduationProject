#pragma once
#include <vector>
#include <chrono>
#include <iomanip>
#include <fstream>
#include <iostream>

#include <opencv2/core.hpp>


/// @brief 所有规划器的接口类
class GlobalPlannerInterface
{
public:
    /// @brief 所有规划器参数的接口类，用于保存规划器使用的所有参数
    struct GlobalPlannerParams
    {
        virtual ~GlobalPlannerParams() = default;
    };

    /// @brief 所有辅助规划器的接口类，用于记录规划耗时等信息，并打印规划结果、参数等信息
    class GlobalPlannerHelper
    {
    public:
        GlobalPlannerHelper(GlobalPlannerInterface * planner) : planner_(planner) {}
        virtual ~GlobalPlannerHelper() = default;

        virtual void showAllInfo(const bool save = false, const std::string & save_dir_path = "") = 0;
        virtual void resetResultInfo() = 0;

    protected:
        GlobalPlannerInterface * planner_ = nullptr;

        virtual std::string paramsInfo() = 0;
        virtual std::string mapInfo() = 0;
        virtual std::string resultInfo() = 0;

        /// @brief 将给定的内容info直接写入file_path中
        /// @param info 待写入的内容
        /// @param file_path 文件路径和文件名字
        /// @return 保存是否成功
        static bool saveInfo(const std::string & info, const std::string & file_path)
        {
            std::ofstream ofs(file_path, std::ios::trunc);
            if (!ofs.is_open())
            {
                return false;
            }
            ofs << info;
            ofs.close();
            return true;
        }

        /// @brief 以字符串形式"%Y-%m-%d %H:%M:%S"获得当前日前和时间。
        /// @return 日期时间字符串
        static std::string daytime()
        {
            const auto now = std::chrono::system_clock::now();              // 当前时间点
            const auto now_tt = std::chrono::system_clock::to_time_t(now);  // 转为时间戳
            const auto now_tm = std::localtime(&now_tt);                    // 根据时间戳和时区返回实际的年月日时分秒

            std::stringstream ss;
            ss << std::put_time(now_tm, "%Y-%m-%d %H:%M:%S");
            return ss.str();
        }
    };

public:
    GlobalPlannerInterface() = default;
    virtual ~GlobalPlannerInterface() = default;

    virtual void initParams(const GlobalPlannerParams & params) = 0;                    // 初始化规划器参数
    virtual bool setMap(const cv::Mat & map) = 0;                                       // 设置栅格地图
    /// @brief 设置地图的参数信息
    /// @param res 分辨率
    /// @param ori_x 原点x
    /// @param ori_y 原点y
    virtual void setMapInfo(const double res, const double ori_x, const double ori_y)
    {
        res_ = res;
        ori_x_ = ori_x;
        ori_y_ = ori_y;
        init_map_info_ = true;
    }
    virtual bool setStartPoint(const int x, const int y) = 0;                           // 设置起点栅格坐标
    virtual bool setStartPoint(const cv::Point2i p) = 0;                                // 设置起点栅格坐标
    virtual bool setEndPoint(const int x, const int y) = 0;                             // 设置终点栅格坐标
    virtual bool setEndPoint(const cv::Point2i p) = 0;                                  // 设置终点栅格坐标

    virtual bool getProcessedMap(cv::Mat & map) = 0;                                                // 处理后的地图，即算法内部真正使用的地图
    virtual bool getRawPath(std::vector<cv::Point2i> & path) = 0;                                   // 原始的以栅格为单位的路径
    virtual bool getSmoothPath(std::vector<cv::Point2d> & path) = 0;                                // 平滑优化后的离散路径，会补齐因栅格坐标系而偏移出的0.5个单位长度的偏差
    virtual void showAllInfo(const bool save = false, const std::string & save_dir_path = "") = 0;  // 用于打印规划的所有信息，包括参数、地图、结果信息
    
protected:
    // 地图属性
    int rows_ = 0;          // 地图行数
    int cols_ = 0;          // 地图列数
    int channels_ = 0;      // 地图通道数，暂未使用
    double res_ = 0.0;      // 地图分辨率，用于将栅格地图中的数据直接转换到真实地图
    double ori_x_ = 0.0;    // 地图原点坐标x，用于将栅格地图中的数据直接转换到真实地图
    double ori_y_ = 0.0;    // 地图原点坐标y，用于将栅格地图中的数据直接转换到真实地图

    // 设置参数
    bool init_map_ = false;         // 地图初始化
    bool init_map_info_ = false;    // 真实地图信息初始化，即res_, ori_x_, ori_y_
    bool init_start_node_ = false;  // 确定规划起点
    bool init_end_node_ = false;    // 确定规划终点
};
