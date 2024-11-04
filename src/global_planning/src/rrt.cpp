#include "global_planning/rrt.h"


// ========================= RRT =========================

bool RRT::setMap(const cv::Mat & map)
{
    // 保证地图合理
    rows_ = map.rows;
    if (rows_ == 0)
    {
        std::cout << "地图的行数/高度不能为0！\n";
        init_map_ = false;
        return false;
    }

    cols_ = map.cols;
    if (cols_ == 0)
    {
        std::cout << "地图的列数/宽度不能为0！\n";
        init_map_ = false;
        return false;
    }

    channels_ = map.channels();
    if (channels_ != 1)
    {
        std::cout << "地图的通道数/层数必须为1！\n";
        init_map_ = false;
        return false;
    }

    map_ = cv::Mat(rows_, cols_, CV_8UC1, cv::Scalar(255));
    for (size_t i = 0; i < rows_; i++)
    {
        for (size_t j = 0; j < cols_; j++)
        {
            const uchar & cost = map.at<uchar>(i, j);
            if (cost <= 50)
            {
                map_.at<uchar>(i, j) = 0;
            }
            else if (cost <= 100)
            {
                map_.at<uchar>(i, j) = 100;
            }
            else
            {
                map_.at<uchar>(i, j) = 255;
            }
        }
    }
    
    // printf("地图设置成功，大小 %d x %d\n", rows_, cols_);
    init_map_ = true;
    return true;
}


// ========================= RRT =========================
