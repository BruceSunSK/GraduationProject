#pragma once
#include <iostream>
#include <random>
#include <vector>

#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>

#include "global_planning/global_planner_interface.h"


/// @brief 用于生成、加载、保存自定义地图，然后调用规划器进行规划并显示规划的结果
class MapGenerator
{
public:
    MapGenerator() = default;
    ~MapGenerator() = default;

    /// @brief 设置要使用的规划器
    /// @param planner 规划器
    void set_planner(GlobalPlannerInterface * const planner)
    {
        planner_ = planner;
        if (rows_ > 0 && cols_ > 0)
        {
            planner_->setMap(grid_map_);
            planner_->setMapInfo(scale_, 0, 0);
        }
    }

    /// @brief 设置规划结果保存的目录路径
    /// @param result_dir_path 目录路径
    void set_result_path(const std::string & result_dir_path)
    {
        result_dir_path_ = result_dir_path;
    }
    
    /// @brief 生成单通道的随机地图，后续可以调用函数扩充阴影
    /// @param rows 地图的行数
    /// @param cols 地图的列数
    /// @param obs_probability 单个栅格为障碍的概率
    void generate_random_map(const int rows = 0, const int cols = 0, const double obs_probability = 0.3)
    {
        // 设置随机数生成器，用于随机生成障碍物
        std::mt19937 random_number_generator;
        random_number_generator.seed();
        std::uniform_int_distribution<int> int_distribution(10, 30); 
        std::uniform_real_distribution<double> double_distribution(0.0f, 1.0f); 

        // 确定地图大小
        rows_ = (rows <= 0 ? int_distribution(random_number_generator) : rows);
        cols_ = (cols <= 0 ? int_distribution(random_number_generator) : cols);
    
        // 根据层数进行地图生成
        grid_map_          = cv::Mat::zeros(rows_, cols_, CV_8UC1);
        grid_map_property_ = cv::Mat::zeros(rows_, cols_, CV_8UC1);
        for (size_t i = 0; i < rows_; i++)
        {
            for (size_t j = 0; j < cols_; j++)
            {
                double grid_p = double_distribution(random_number_generator);
                bool is_obs = grid_p < obs_probability;
                grid_map_.at<uchar>(i, j) = (is_obs ? 100 : 0);   // 100为障碍物 0为可通行区域
                grid_map_property_.at<uchar>(i, j) = (is_obs ? GridType::OBSTACLE : GridType::GROUND);
            }
        }

        if (planner_ != nullptr)
        {
            planner_->setMap(grid_map_);
            planner_->setMapInfo(scale_, 0, 0);
        }
    }

    /// @brief 生成单通道的空白地图，后续可以手动添加障碍物、调用函数扩充阴影
    /// @param rows 地图的行数
    /// @param cols 地图的列数
    void generate_blank_map(const int rows, const int cols)
    {
        if (rows <= 0)
        {
            std::cout << "地图的行数/高度必须大于0！\n";
            return;
        }
        
        if (cols <= 0)
        {
            std::cout << "地图的列数/宽度必须大于0！\n";
            return;
        }

        rows_ = rows;
        cols_ = cols;
        grid_map_          = cv::Mat::zeros(rows_, cols_, CV_8UC1);
        grid_map_property_ = cv::Mat::zeros(rows_, cols_, CV_8UC1); // 默认全部为GROUND类型

        if (planner_ != nullptr)
        {
            planner_->setMap(grid_map_);
            planner_->setMapInfo(scale_, 0, 0);
        }
    }

    /// @brief 添加单点障碍物
    /// @param point 点的坐标
    void add_obstacles(const cv::Point2i & point)
    {
        grid_map_.at<uchar>(point) = 100;
        grid_map_property_.at<uchar>(point) = GridType::OBSTACLE;
    }

    /// @brief 添加多个离散点障碍物
    /// @param points 所有点的集合
    void add_obstacles(const std::vector<cv::Point2i> & points)
    {
        for (auto && p : points)
        {            
            grid_map_.at<uchar>(p) = 100;
            grid_map_property_.at<uchar>(p) = GridType::OBSTACLE;
        }
    }

    /// @brief 添加指定两点连线间的障碍物
    /// @param point1 第一个点的坐标
    /// @param point2 第二个点的坐标
    void add_obstacles(const cv::Point2i & point1, const cv::Point2i & point2)
    {
        cv::line(grid_map_, point1, point2, 100, 1);
        cv::line(grid_map_property_, point1, point2, GridType::OBSTACLE, 1);
    }

    /// @brief 添加指定矩形区域内的障碍物
    /// @param rectangle 矩形区域
    /// @param filled 是否对内部进行填充
    void add_obstacles(const cv::Rect & rectangle, const bool filled = false)
    {
        cv::rectangle(grid_map_, rectangle, 100, filled ? -1 : 1);
        cv::rectangle(grid_map_property_, rectangle, GridType::OBSTACLE, filled ? -1 : 1);
    }

    /// @brief 添加指定圆形区域内的障碍物
    /// @param center 圆心坐标
    /// @param radius 半径
    /// @param filled 是否对内部进行填充
    void add_obstacles(const cv::Point2i center, const int radius, const bool filled = false)
    {
        cv::circle(grid_map_, center, radius, 100, filled ? -1 : 1);
        cv::circle(grid_map_property_, center, radius, GridType::OBSTACLE, filled ? -1 : 1);
    }

    /// @brief 保存地图数据（非图片效果）到目标地址
    /// @param path_and_name 保存的路径及文件名
    /// @return 是否保存成功
    bool save_map(const std::string & path_and_name)
    {
        if (rows_ == 0 || cols_ == 0 || path_and_name.empty())
        {
            return false;
        }

        return cv::imwrite(path_and_name, grid_map_);
    }

    /// @brief 加载地图数据
    /// @param path_and_name 加载的路径及文件名
    void load_map(const std::string & path_and_name)
    {
        grid_map_ = cv::imread(path_and_name, cv::ImreadModes::IMREAD_GRAYSCALE);
        rows_ = grid_map_.rows;
        cols_ = grid_map_.cols;

        grid_map_property_ = cv::Mat::zeros(rows_, cols_, CV_8UC1);
        for (size_t i = 0; i < rows_; i++)
        {
            for (size_t j = 0; j < cols_; j++)
            {
                grid_map_property_.at<uchar>(i, j) = (grid_map_.at<uchar>(i, j) == 100 ? GridType::OBSTACLE : GridType::GROUND);
            }
        }

        if (planner_ != nullptr)
        {
            planner_->setMap(grid_map_);
            planner_->setMapInfo(scale_, 0, 0);
        }
    }

    /// @brief 展示当前的栅格地图，并且调用实时规划效果。用于最终显示效果
    /// @param winname 创建的窗口的名字
    /// @param scale 对栅格进行缩放的比例
    void show_map(const cv::String & winname, const int scale = 5)
    {
        scale_ = scale;
        planner_->setMapInfo(scale_, 0, 0);

        show_image_ = cv::Mat::zeros(rows_ * scale_, cols_ * scale_, CV_8UC3);
        cv::Mat processed_map;
        planner_->getProcessedMap(processed_map);
        for (size_t i = 0; i < rows_; i++)
        {
            for (size_t j = 0; j < cols_; j++)
            {
                uchar & value = processed_map.at<uchar>(i, j);
                uchar new_value = (100 - value) / 100.0 * 255;
                set_show_image_color(i, j, cv::Vec3b(new_value, new_value, new_value));
            }
        }

        cv::namedWindow(winname, cv::WindowFlags::WINDOW_NORMAL | cv::WindowFlags::WINDOW_KEEPRATIO);
        cv::setMouseCallback(winname, on_mouse_callback, this); // 设置鼠标回调
        while (true)
        {
            cv::imshow(winname, show_image_);
            int key = cv::waitKey(100) & 0xff;
            if (key == 27 || key == 'q') // 按esc或q退出
            {
                break;
            }
        }
    }

    /// @brief 用于测试最终每栅格颜色
    void color_test()
    {
        cv::Mat color_test_image = cv::Mat::zeros(1, 8, CV_8UC3);
        color_test_image.at<cv::Vec3b>(0, 0)[0] = GridType::GROUND;
        color_test_image.at<cv::Vec3b>(0, 1)[0] = GridType::OBSTACLE;
        color_test_image.at<cv::Vec3b>(0, 2)[0] = GridType::OPEN;
        color_test_image.at<cv::Vec3b>(0, 3)[0] = GridType::CLOSE;
        color_test_image.at<cv::Vec3b>(0, 4)[0] = GridType::START;
        color_test_image.at<cv::Vec3b>(0, 5)[0] = GridType::END;
        color_test_image.at<cv::Vec3b>(0, 6)[0] = GridType::PATH;
        color_test_image.at<cv::Vec3b>(0, 7)[0] = GridType::UNKNOWN;

        for (size_t i = 0; i < 8; i++)
        {
            color_test_image.at<cv::Vec3b>(0, i) = get_grid_color(static_cast<GridType>(color_test_image.at<cv::Vec3b>(0, i)[0]));
        }

        cv::namedWindow("Grid Color Test", cv::WindowFlags::WINDOW_NORMAL | cv::WindowFlags::WINDOW_KEEPRATIO);
        cv::imshow("Grid Color Test", color_test_image);
        cv::waitKey();
    }

private:
    int rows_ = 0;
    int cols_ = 0;
    cv::Mat grid_map_;          // 用于储存栅格原始数据
    cv::Mat grid_map_property_; // 用于储存栅格附加的特征信息
    cv::Mat show_image_;        // 用于最终结果可视化
    int scale_ = 4;             // 用于将原始栅格放大到可视化图形中，也方便后续绘制路径
    GlobalPlannerInterface * planner_ = nullptr; // 规划器
    std::string result_dir_path_;   // 规划结果保存目录路径

    /// @brief 栅格的属性
    enum GridType : uint8_t
    {
        GROUND,             // 普通地面，即还未进行搜索
        OBSTACLE,           // 障碍物
        OPEN,               // 加入open集合
        CLOSE,              // 加入close集合
        START,              // 起点
        END,                // 终点
        PATH,               // 最终路径
        UNKNOWN             // 一般是未探索区域
    };

    /// @brief 输入栅格地图中的某个点(col, row)/(x, y)和颜色，将对应的可视化图像的对应区域设定成所需颜色
    /// @param row 所处的行
    /// @param col 所处的列
    /// @param color 要设置的颜色
    void set_show_image_color(const int row, const int col, const cv::Vec3b & color)
    {
        cv::rectangle(show_image_, cv::Rect(col * scale_, row * scale_, scale_, scale_), color, -1);
    }

    /// @brief 输入栅格地图中的某个点(col, row)/(x, y)和颜色，将对应的可视化图像的对应区域设定成所需颜色
    /// @param point 点的坐标
    /// @param color 要设置的颜色
    void set_show_image_color(const cv::Point2i & point, const cv::Vec3b & color)
    {
        cv::rectangle(show_image_, cv::Rect(point.x * scale_, point.y * scale_, scale_, scale_), color, -1);
    }

    /// @brief 针对小的离散点进行绘制颜色
    /// @param point 点的坐标
    /// @param color 要设置的颜色
    void set_show_image_color(const cv::Point2d & point, const cv::Vec3b & color)
    {
        cv::circle(show_image_, point, scale_ / 3, color, -1);
    }

    /// @brief 根据栅格种类映射到对应的颜色
    /// @param type 栅格种类
    /// @return 对应的颜色
    static cv::Vec3b get_grid_color(const GridType type)
    {
        switch (type)
        {
        case GridType::GROUND:      // 白色
            return cv::Vec3b(255, 255, 255);
        case GridType::OBSTACLE:    // 黑色
            return cv::Vec3b(0, 0, 0);
        case GridType::OPEN:        // 宝蓝色
            return cv::Vec3b(255, 105, 65);
        case GridType::CLOSE:       // 蓝色
            return cv::Vec3b(255, 0, 0);
        case GridType::START:       // 红色
            return cv::Vec3b(0, 0, 200);
        case GridType::END:         // 粉色
            return cv::Vec3b(255, 40, 255);
        case GridType::PATH:        // 浅绿色
            return cv::Vec3b(0, 190, 0);   
        default:                    // 墨绿色
            return cv::Vec3b(0, 90, 0);
        }
    }

    /// @brief 根据鼠标的点击进行回调操作。实现依次点击的点为起点和终点。设置起点和终点后规划路径并显示
    /// @param params 由于在类内部操作，opencv不支持类成员函数绑定，因此需要传入this指针
    static void on_mouse_callback(int event, int x, int y, int flags, void * params)
    {
        static int left_click_count = 0;
        static cv::Mat grid_map_property_clone, show_image_clone;
        static MapGenerator * obj = reinterpret_cast<MapGenerator *>(params);
        if (left_click_count == 0)
        {
            obj->grid_map_.copyTo(grid_map_property_clone);
            obj->show_image_.copyTo(show_image_clone);
        }

        if (event == cv::MouseEventTypes::EVENT_LBUTTONDOWN && obj->planner_ != nullptr) 
        {
            int grid_x = x / obj->scale_;
            int grid_y = y / obj->scale_;
            if (left_click_count % 2 == 0)
            {
                if (obj->planner_->setStartPoint(grid_x, grid_y))
                {
                    // 每次复原
                    grid_map_property_clone.copyTo(obj->grid_map_property_);
                    show_image_clone.copyTo(obj->show_image_);

                    // 标记起点
                    obj->grid_map_property_.at<cv::Vec2b>(grid_y, grid_x)[1] = GridType::START;
                    obj->set_show_image_color(grid_y, grid_x, get_grid_color(GridType::START));
                    left_click_count++;
                }
            }
            else
            {
                if (obj->planner_->setEndPoint(grid_x, grid_y))
                {
                    // 标记终点
                    obj->grid_map_property_.at<cv::Vec2b>(grid_y, grid_x)[1] = GridType::END;
                    obj->set_show_image_color(grid_y, grid_x, get_grid_color(GridType::END));
                    left_click_count++;

                    // 调用规划结果
                    std::vector<cv::Point2i> path;
                    if (obj->planner_->getRawPath(path))
                    {
                        // 1. 直接绘制原始路径
                        for (size_t i = 1; i < path.size() - 1; i++)
                        {
                            cv::Point2i & p = path[i];
                            obj->grid_map_property_.at<uchar>(p.y, p.x) = GridType::CLOSE;
                            obj->set_show_image_color(p, get_grid_color(GridType::CLOSE));
                        }

                        // 2. 绘制贝塞尔曲线平滑后的路径
                        std::vector<cv::Point2d> smooth_path;
                        obj->planner_->getSmoothPath(smooth_path);
                        for (size_t i = 0; i < smooth_path.size(); i++)
                        {
                            cv::Point2d & p = smooth_path[i];
                            obj->set_show_image_color(p, get_grid_color(GridType::PATH));
                        }

                        obj->planner_->showAllInfo(true, obj->result_dir_path_);
                        std::cout << "路径规划成功，raw路径长度：" << path.size() << "，smooth路径长度：" << smooth_path.size() << std::endl << std::endl;
                    }
                    else
                    {
                        std::cerr << "路径规划失败！" << std::endl << std::endl;
                    }
                }  
            }
        }
    }
};

