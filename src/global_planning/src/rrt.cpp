#include "global_planning/rrt.h"


// ========================= RRT::RRTHelper =========================

void RRT::RRTHelper::showAllInfo(const bool save, const std::string & save_dir_path) const
{
    const std::string dt = daytime();

    std::stringstream info;
    info << "--------------- [RRT Info] ---------------\n"
         << "Daytime: " << dt << "\tAuthor: BruceSun\n\n"
         << paramsInfo() << std::endl
         << mapInfo() << std::endl
         << resultInfo()
         << "--------------- [RRT Info] ---------------\n\n";
    std::cout << info.str();

    if (save)
    {
        std::string file_path = save_dir_path;
        if (file_path.back() != '/')
        {
            file_path.push_back('/');
        }
        file_path += "RRT/";
        file_path += (dt + " RRT_All_Info.txt");

        if (saveInfo(info.str(), file_path))
        {
            std::cout << "[RRT Info]: All Info Has Saved to " << file_path << std::endl;
        }
        else
        {
            std::cerr << "[RRT Info]: All Info Failed to Save to " << file_path << std::endl;
        }
    }
}


std::string RRT::RRTHelper::paramsInfo() const
{
    const RRT * rrt_planner = dynamic_cast<const RRT *>(planner_);
    const RRTParams & rrt_params = rrt_planner->params_;

    std::stringstream params_info;
    params_info << "[Params Info]:\n";
    PRINT_STRUCT(params_info, rrt_params);
    return params_info.str();
}

std::string RRT::RRTHelper::mapInfo() const
{
    const RRT * rrt_planner = dynamic_cast<const RRT *>(planner_);

    std::stringstream map_info;
    map_info << "[Map Info]:\n"
             << "  rows: " << rrt_planner->rows_ << std::endl
             << "  cols: " << rrt_planner->cols_ << std::endl
             << "  resolution: " << rrt_planner->res_ << std::endl
             << "  start point: " << rrt_planner->start_point_ << std::endl
             << "  end point:   " << rrt_planner->end_point_ << std::endl;
    return map_info.str();
}

std::string RRT::RRTHelper::resultInfo() const
{
    std::stringstream result_info;
    result_info << "[Result Info]:\n";
    PRINT_STRUCT(result_info, search_result);
    return result_info.str();
}
// ========================= RRT::RRTHelper =========================

// ========================= RRT =========================

void RRT::initParams(const GlobalPlannerParams & params)
{
    const RRTParams & p = dynamic_cast<const RRTParams &>(params);
    params_ = p;
}

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
            if (cost <= params_.map_params.OBSTACLE_THRESHOLD)
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

bool RRT::setStartPoint(const double x, const double y)
{
    if (init_map_info_ == false)
    {
        std::cout << "设置起点前需设置地图信息！" << '\n';
        init_start_node_ = false;
        return false;
    }

    double x_grid_double = (x - ori_x_) / res_;
    double y_grid_double = (y - ori_y_) / res_;
    int x_grid = static_cast<int>(x_grid_double);
    int y_grid = static_cast<int>(y_grid_double);

    if ((x_grid >= 0 && x_grid < cols_ && y_grid >= 0 && y_grid < rows_) == false)
    {
        std::cout << "起点必须设置在地图内部！" << '\n';
        init_start_node_ = false;
        return false;
    }

    if (map_.at<uchar>(y_grid, x_grid) >= params_.map_params.OBSTACLE_THRESHOLD)
    {
        std::cout << "起点必须设置在非障碍物处！" << '\n';
        init_start_node_ = false;
        return false;
    }

    // printf("起点设置成功，位置：(%d, %d)\n", x_grid, y_grid);
    start_point_.x = x_grid_double;
    start_point_.y = y_grid_double;
    init_start_node_ = true;
    return true;
}

bool RRT::setStartPoint(const cv::Point2d & p)
{
    return setStartPoint(p.x, p.y);
}

bool RRT::setEndPoint(const double x, const double y)
{
    if (init_map_info_ == false)
    {
        std::cout << "设置终点前需设置地图信息！" << '\n';
        init_end_node_ = false;
        return false;
    }

    double x_grid_double = (x - ori_x_) / res_;
    double y_grid_double = (y - ori_y_) / res_;
    int x_grid = static_cast<int>(x_grid_double);
    int y_grid = static_cast<int>(y_grid_double);

    if ((x_grid >= 0 && x_grid < cols_ && y_grid >= 0 && y_grid < rows_) == false)
    {
        std::cout << "终点必须设置在地图内部！\n";
        init_end_node_ = false;
        return false;
    }

    if (map_.at<uchar>(y_grid, x_grid) >= params_.map_params.OBSTACLE_THRESHOLD)
    {
        std::cout << "终点必须设置在非障碍物处！\n";
        init_end_node_ = false;
        return false;
    }

    // printf("终点设置成功，位置：(%d, %d)\n", x_grid, y_grid);
    end_point_.x = x_grid_double;
    end_point_.y = y_grid_double;
    init_end_node_ = true;
    return true;
}

bool RRT::setEndPoint(const cv::Point2d & p)
{
    return setEndPoint(p.x, p.y);
}

bool RRT::getProcessedMap(cv::Mat & map) const
{
    if (!init_map_)
    {
        std::cout << "当前无有效地图！\n";
        return false;
    }

    map = map_.clone();
    return true;
}

bool RRT::getPath(std::vector<cv::Point2d> & path, std::vector<std::vector<cv::Point2d>> & auxiliary_info)
{
    if (!(init_map_ && init_start_node_ && init_end_node_ && init_map_info_))
    {
        std::cout << "请先设置地图、起点和终点！\n";
        return false;
    }

    // 设置随机数
    std::mt19937 random_number_generator(static_cast<uint_fast32_t>(std::time(nullptr)));
    std::uniform_real_distribution<double> dis_row(0.0, rows_);     // 设置随机数生成器，用于生成随机节点，离散值，保证能取到所有情况
    std::uniform_real_distribution<double> dis_col(0.0, cols_);     // 设置随机数生成器，用于生成随机节点，离散值，保证能取到所有情况
    std::uniform_real_distribution<double> dis_goal(0.0, 1.0);      // 设置随机数生成器，用于判断是否直接取到终点

    // 将开始节点加入列表
    helper_.resetResultInfo();
    auto start_time = std::chrono::steady_clock::now();
    TreeNode * start_node = new TreeNode(start_point_, nullptr);
    tree_list_.push_back(std::move(start_node));

    // 开始迭代
    for (size_t it = 0; it < params_.sample_params.ITERATOR_TIMES; it++)
    {
        // 生成随机节点
        cv::Point2d random_point;
        if (dis_goal(random_number_generator) < params_.sample_params.GOAL_SAMPLE_RATE)    // 直接选取终点为目标点
        {
            random_point.x = end_point_.x;
            random_point.y = end_point_.y;
        }
        else    // 随机生成任意位置节点
        {
            random_point.x = dis_col(random_number_generator);
            random_point.y = dis_row(random_number_generator);
        }

        // 寻找当前随机节点的最近现有节点
        const size_t nearest_index = nearest_node_index(random_point);

        // 沿最近节点和随机节点方向创建新节点
        const cv::Point2d & nearest_point = tree_list_[nearest_index]->pos;
        const double angle = std::atan2(random_point.y - nearest_point.y, random_point.x - nearest_point.x);
        cv::Point2d new_point(nearest_point.x + params_.sample_params.STEP_SIZE / res_ * std::cos(angle),
                              nearest_point.y + params_.sample_params.STEP_SIZE / res_ * std::sin(angle));
        bool finish = false;
        if (std::hypot(new_point.x - end_point_.x, new_point.y - end_point_.y) < params_.sample_params.GOAL_DIS_TOLERANCE / res_)
        {
            new_point = end_point_;
            finish = true;
        }

        // 新节点的区域判断
        if (!is_inside_map(new_point))  // 当前点不在地图内部，直接进行下轮迭代
        {
            continue;
        }

        // 新节点的碰撞检测
        if (check_collision(new_point, nearest_point))  // 发生碰撞，直接进行下轮迭代
        {
            continue;
        }

        // 添加新节点
        // 对于需不需要补齐0.5，rrt和astar不同
        // 1.rrt是将真实点转换成离散栅格点（也就是0.0~1.0对应栅格坐标0； 1.0~2.0对应栅格坐标1），然后所有计算中间量和结果都是都是离散点。
        //   针对这个离散点判断栅格代价值时，直接向下取整（即截断小数点）就能转换成栅格坐标，从而确定离散点所在栅格坐标。
        //   然后将离散点转换回真实坐标时，是离散点到离散点，因此不需要0.5补齐。
        // 2.astar是将真实点转换成整数栅格点，在这个过程和rrt一样，都是截断小数点就能判断出离散点对应的栅格点坐标。
        //   不同的是，astar最终是将栅格点转换回真实点，如果不补齐0.5就会偏移。
        //   例如：栅格点(0, 0)如果直接转换成离散点(0.0, 0.0)其实是该栅格的左上角角点(opencv坐标系)，但最好用栅格中心点指代该点位置，因此需要偏移0.5
        TreeNode * new_node = new TreeNode(new_point, tree_list_[nearest_index]);
        helper_.search_result.cur_points.push_back(cv::Point2d(new_node->pos.x * res_ + ori_x_, new_node->pos.y * res_ + ori_y_));
        helper_.search_result.par_points.push_back(cv::Point2d(new_node->parent->pos.x * res_ + ori_x_, new_node->parent->pos.y * res_ + ori_y_));
        tree_list_.push_back(std::move(new_node));
        if (finish)
        {
            // 保存结果路径
            path.clear();
            TreeNode * n = tree_list_.back();
            while (n)
            {
                path.push_back(cv::Point2d(n->pos.x * res_ + ori_x_,
                                           n->pos.y * res_ + ori_y_));
                n = n->parent;
            }
            std::reverse(path.begin(), path.end());

            // 保存结果信息
            auto end_time = std::chrono::steady_clock::now();
            helper_.search_result.node_nums = tree_list_.size();
            helper_.search_result.node_counter = it + 1;                                    // +1表示实际执行的第i次迭代
            helper_.search_result.path_length = path.size();                                // 路径长度
            helper_.search_result.cost_time = (end_time - start_time).count() / 1000000.0;  // 算法耗时 ms
            auxiliary_info.push_back(helper_.search_result.cur_points);
            auxiliary_info.push_back(helper_.search_result.par_points);

            // 释放内存
            for (size_t i = 0; i < tree_list_.size(); i++)
            {
                delete tree_list_[i];
            }
            tree_list_.clear();
            return true;
        }
    }

    std::cout << "RRT求解失败！ 超出迭代次数：" << params_.sample_params.ITERATOR_TIMES << " \n";
    for (size_t i = 0; i < tree_list_.size(); i++)
    {
        delete tree_list_[i];
    }
    tree_list_.clear();
    return false;
}


size_t RRT::nearest_node_index(const cv::Point2d & rd_pt) const
{
    size_t index = 0;
    double dis = std::numeric_limits<double>::max();

    for (size_t i = 0; i < tree_list_.size(); i++)
    {
        const cv::Point2d & node = tree_list_[i]->pos;
        const double dis_temp = std::hypot(node.x - rd_pt.x, node.y - rd_pt.y);
        if (dis_temp < dis)
        {
            dis = dis_temp;
            index = i;
        }
    }
    return index;
}

bool RRT::is_inside_map(const cv::Point2d & pt) const
{
    // 待判断点为离散点，而非栅格点，因此上限可以取到cols_和rows_
    return pt.x >= 0 && pt.x <= cols_ && pt.y >= 0 && pt.y <= rows_;
}

bool RRT::check_collision(const cv::Point2d & pt1, const cv::Point2d & pt2) const
{
    const cv::Point2i pt1_grid(static_cast<int>(pt1.x), static_cast<int>(pt1.y));
    const cv::Point2i pt2_grid(static_cast<int>(pt2.x), static_cast<int>(pt2.y));   
    const std::vector<cv::Point2i> pts = PathSimplification::Bresenham(pt1_grid, pt2_grid, 2);
    for (const cv::Point2i & p : pts)
    {
        if (map_.at<uchar>(p) >= params_.map_params.OBSTACLE_THRESHOLD)
        {
            return true;
        }
    }
    return false;
}
// ========================= RRT =========================
