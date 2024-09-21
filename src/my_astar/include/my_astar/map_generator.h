#pragma once
#include <iostream>
#include <random>
#include <vector>

#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>

#include "my_astar/global_planning_interface.h"
#include "my_astar/debug.h"

class MapGenerator
{
public:
    MapGenerator()
    {
    }
    ~MapGenerator()
    {
    }

    void set_planner(GlobalPlannerInterface * const planner)
    {
        planner_ = planner;
        if (rows_ > 0 && cols_ > 0)
        {
            planner_->setMap(grid_map_);
            planner_->setMapInfo(scale_, 0, 0);
        }
    }

    // 生成单通道的随机地图，后续可以调用函数扩充阴影
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

    // 生成单通道的空白地图，后续可以手动添加障碍物、调用函数扩充阴影
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

    // 添加单点障碍物
    void add_obstacles(const cv::Point2i & point)
    {
        grid_map_.at<uchar>(point) = 100;
        grid_map_property_.at<uchar>(point) = GridType::OBSTACLE;
    }

    // 添加多个离散点障碍物
    void add_obstacles(const std::vector<cv::Point2i> & points)
    {
        for (auto && p : points)
        {            
            grid_map_.at<uchar>(p) = 100;
            grid_map_property_.at<uchar>(p) = GridType::OBSTACLE;
        }
    }

    // 添加指定两点间的障碍物
    void add_obstacles(const cv::Point2i & point1, const cv::Point2i & point2)
    {
        cv::line(grid_map_, point1, point2, 100, 1);
        cv::line(grid_map_property_, point1, point2, GridType::OBSTACLE, 1);
    }

    // 添加指定矩形区域内的障碍物
    void add_obstacles(const cv::Rect & rectangle, const bool filled = false)
    {
        cv::rectangle(grid_map_, rectangle, 100, filled ? -1 : 1);
        cv::rectangle(grid_map_property_, rectangle, GridType::OBSTACLE, filled ? -1 : 1);
    }

    // 添加指定圆形区域内的障碍物
    void add_obstacles(const cv::Point2i center, const int radius, const bool filled = false)
    {
        cv::circle(grid_map_, center, radius, 100, filled ? -1 : 1);
        cv::circle(grid_map_property_, center, radius, GridType::OBSTACLE, filled ? -1 : 1);
    }

    // 保存地图数据（非图片效果）
    bool save_map(const std::string & path_and_name)
    {
        if (rows_ == 0 || cols_ == 0 || path_and_name.empty())
        {
            return false;
        }

        return cv::imwrite(path_and_name, grid_map_);
    }

    // 加载地图数据
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

    // 根据当前障碍物的位置进行膨胀，将二值化（0和100）的地图变成离散值。
    // 5x5 范围内的代价值系数与距离成反比。即距离为sqrt(2)的格子代价值即为 1/sqrt(2)*COST，COST为中心代价值
    void expand_map()
    {
        static const cv::Mat kernel = (cv::Mat_<double>(15, 15) << 0.10102, 0.10847, 0.11625, 0.12403, 0.13131, 0.13736, 0.14142, 0.14286, 0.14142, 0.13736, 0.13131, 0.12403, 0.11625, 0.10847, 0.10102, 
                                                                  0.10847, 0.11785, 0.12804, 0.13868, 0.14907, 0.15811, 0.16440, 0.16667, 0.16440, 0.15811, 0.14907, 0.13868, 0.12804, 0.11785, 0.10847, 
                                                                  0.11625, 0.12804, 0.14142, 0.15617, 0.17150, 0.18570, 0.19612, 0.20000, 0.19612, 0.18570, 0.17150, 0.15617, 0.14142, 0.12804, 0.11625, 
                                                                  0.12403, 0.13868, 0.15617, 0.17678, 0.20000, 0.22361, 0.24254, 0.25000, 0.24254, 0.22361, 0.20000, 0.17678, 0.15617, 0.13868, 0.12403, 
                                                                  0.13131, 0.14907, 0.17150, 0.20000, 0.23570, 0.27735, 0.31623, 0.33333, 0.31623, 0.27735, 0.23570, 0.20000, 0.17150, 0.14907, 0.13131, 
                                                                  0.13736, 0.15811, 0.18570, 0.22361, 0.27735, 0.35355, 0.44721, 0.50000, 0.44721, 0.35355, 0.27735, 0.22361, 0.18570, 0.15811, 0.13736, 
                                                                  0.14142, 0.16440, 0.19612, 0.24254, 0.31623, 0.44721, 0.70711, 1.00000, 0.70711, 0.44721, 0.31623, 0.24254, 0.19612, 0.16440, 0.14142, 
                                                                  0.14286, 0.16667, 0.20000, 0.25000, 0.33333, 0.50000, 1.00000, 1.00000, 1.00000, 0.50000, 0.33333, 0.25000, 0.20000, 0.16667, 0.14286, 
                                                                  0.14142, 0.16440, 0.19612, 0.24254, 0.31623, 0.44721, 0.70711, 1.00000, 0.70711, 0.44721, 0.31623, 0.24254, 0.19612, 0.16440, 0.14142, 
                                                                  0.13736, 0.15811, 0.18570, 0.22361, 0.27735, 0.35355, 0.44721, 0.50000, 0.44721, 0.35355, 0.27735, 0.22361, 0.18570, 0.15811, 0.13736, 
                                                                  0.13131, 0.14907, 0.17150, 0.20000, 0.23570, 0.27735, 0.31623, 0.33333, 0.31623, 0.27735, 0.23570, 0.20000, 0.17150, 0.14907, 0.13131, 
                                                                  0.12403, 0.13868, 0.15617, 0.17678, 0.20000, 0.22361, 0.24254, 0.25000, 0.24254, 0.22361, 0.20000, 0.17678, 0.15617, 0.13868, 0.12403, 
                                                                  0.11625, 0.12804, 0.14142, 0.15617, 0.17150, 0.18570, 0.19612, 0.20000, 0.19612, 0.18570, 0.17150, 0.15617, 0.14142, 0.12804, 0.11625, 
                                                                  0.10847, 0.11785, 0.12804, 0.13868, 0.14907, 0.15811, 0.16440, 0.16667, 0.16440, 0.15811, 0.14907, 0.13868, 0.12804, 0.11785, 0.10847, 
                                                                  0.10102, 0.10847, 0.11625, 0.12403, 0.13131, 0.13736, 0.14142, 0.14286, 0.14142, 0.13736, 0.13131, 0.12403, 0.11625, 0.10847, 0.10102);
        if (rows_ == 0 || cols_ == 0)
        {
            return;
        }

        ros::Time start = ros::Time::now();
        cv::Mat src = grid_map_.clone();
        int kernel_half_width = kernel.cols / 2;
        for (int i = 0; i < rows_; i++)
        {
            for (int j = 0; j < cols_; j++)
            {
                uchar & cost = src.at<uchar>(i, j);
                for (int m = -kernel_half_width; m <= kernel_half_width; m++)
                {
                    if (i + m < 0)
                    {
                        continue;
                    }
                    if (i + m >= rows_)
                    {
                        break;
                    }
                    
                    for (int n = -kernel_half_width; n <= kernel_half_width; n++)
                    {
                        if (j + n < 0)
                        {
                            continue;
                        }
                        if (j + n >= cols_)
                        {
                            break;
                        }
                        
                        uchar new_cost = static_cast<uchar>(kernel.at<double>(m + kernel_half_width, n + kernel_half_width) * cost);
                        if (grid_map_.at<uchar>(i + m, j + n) < new_cost)
                        {
                            grid_map_.at<uchar>(i + m, j + n) = new_cost;
                            grid_map_property_.at<uchar>(i + m, j + n) = GridType::EXPANDED_OBSTACLE;
                        }
                    }
                }
            }
        }

        ros::Time end = ros::Time::now();
        DEBUG_PRINT((end - start).toSec());
    }

    // 展示当前的栅格地图，并且调用实时规划效果
    void show_map(const cv::String & winname, const int scale = 5)
    {
        scale_ = scale;
        planner_->setMapInfo(scale_, 0, 0);

        show_image_ = cv::Mat::zeros(rows_ * scale_, cols_ * scale_, CV_8UC3);
        for (size_t i = 0; i < rows_; i++)
        {
            for (size_t j = 0; j < cols_; j++)
            {
                GridType type = static_cast<GridType>(grid_map_property_.at<uchar>(i, j));
                if (type == GridType::EXPANDED_OBSTACLE)
                {
                    uchar & value = grid_map_.at<uchar>(i, j);
                    uchar new_value = (100 - value) / 100.0 * 255;
                    set_show_image_color(i, j, cv::Vec3b(new_value, new_value, new_value));
                }
                else
                {
                    set_show_image_color(i, j, get_grid_color(type));
                }
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

    // 用于测试最终每栅格颜色
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


    enum GridType : uint8_t
    {
        GROUND,             // 普通地面，即还未进行搜索
        OBSTACLE,           // 障碍物
        EXPANDED_OBSTACLE,  // 扩展出的障碍物
        OPEN,               // 加入open集合
        CLOSE,              // 加入close集合
        START,              // 起点
        END,                // 终点
        PATH,               // 最终路径
        UNKNOWN             // 一般是未探索区域
    };

    // 输入栅格地图中的某个点(col, row)/(x, y)和颜色，将对应的可视化图像的对应区域设定成所需颜色
    void set_show_image_color(const int row, const int col, const cv::Vec3b & color)
    {
        cv::rectangle(show_image_, cv::Rect(col * scale_, row * scale_, scale_, scale_), color, -1);
    }

    // 以点的数据形式实现上述功能
    void set_show_image_color(const cv::Point2i & point, const cv::Vec3b & color)
    {
        cv::rectangle(show_image_, cv::Rect(point.x * scale_, point.y * scale_, scale_, scale_), color, -1);
    }

    // 针对小的离散点进行绘制
    void set_show_image_color(const cv::Point2d & point, const cv::Vec3b & color)
    {
        cv::circle(show_image_, point, scale_ / 3, color, -1);
    }

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

