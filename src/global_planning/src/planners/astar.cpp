#include "global_planning/planners/astar.h"


// ========================= Astar::AstarHelper =========================

std::string Astar::AstarHelper::paramsInfo() const
{
    const Astar * astar_planner = dynamic_cast<const Astar *>(planner_);
    const AstarParams & astar_params = astar_planner->params_;

    std::stringstream params_info;
    params_info << "[Params Info]:\n";
    PRINT_STRUCT(params_info, astar_params);
    return params_info.str();
}

std::string Astar::AstarHelper::mapInfo() const
{
    const Astar * astar_planner = dynamic_cast<const Astar *>(planner_);

    std::stringstream map_info;
    map_info << "[Map Info]:\n"
             << "  rows: " << astar_planner->rows_ << std::endl
             << "  cols: " << astar_planner->cols_ << std::endl
             << "  resolution: " << astar_planner->res_ << std::endl
             << "  start point: " << astar_planner->start_node_->point << std::endl
             << "  end point:   " << astar_planner->end_node_->point << std::endl;
    return map_info.str();
}

std::string Astar::AstarHelper::resultInfo() const
{
    std::stringstream result_info;
    result_info << "[Result Info]:\n";
    PRINT_STRUCT(result_info, search);
    return result_info.str();
}
// ========================= Astar::AstarHelper =========================


// ========================= Astar =========================

REGISTER_ENUM_BODY(AHeuristicsType,
                   REGISTER_MEMBER(AHeuristicsType::None),
                   REGISTER_MEMBER(AHeuristicsType::Manhattan),
                   REGISTER_MEMBER(AHeuristicsType::Euclidean),
                   REGISTER_MEMBER(AHeuristicsType::Chebyshev),
                   REGISTER_MEMBER(AHeuristicsType::Octile));

void Astar::initParams(const GlobalPlannerParams & params)
{
    const AstarParams & p = dynamic_cast<const AstarParams &>(params);
    params_ = p;
}

bool Astar::setMap(const cv::Mat & map)
{
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

    map_.clear();
    map_.resize(rows_);
    for (int i = 0; i < rows_; i++)
    {
        std::vector<Node> row(cols_);
        for (int j = 0; j < cols_; j++)
        {
            Node node;
            node.point.x = j;
            node.point.y = i;
            node.cost = (map.at<uchar>(i, j) != 255 ?
                            (map.at<uchar>(i, j) >= params_.map.OBSTACLE_THRESHOLD ? 100 : 0) : 255);
            node.type = Node::NodeType::UNKNOWN;
            row[j] = std::move(node);
        }
        map_[i] = std::move(row);
    }

    // printf("地图设置成功，大小 %d x %d\n", rows_, cols_);
    init_map_ = true;
    return true;
}

bool Astar::setStartPoint(const double x, const double y)
{
    if (init_map_info_ == false)
    {
        std::cout << "设置起点前需设置地图信息！" << '\n';
        init_start_point_ = false;
        return false;
    }

    int x_grid = static_cast<int>((x - ori_x_) / res_);
    int y_grid = static_cast<int>((y - ori_y_) / res_);

    if ((x_grid >= 0 && x_grid < cols_ && y_grid >= 0 && y_grid < rows_) == false)
    {
        std::cout << "起点必须设置在地图内部！" << '\n';
        init_start_point_ = false;
        return false;
    }

    if (map_[y_grid][x_grid].cost >= params_.map.OBSTACLE_THRESHOLD)
    {
        std::cout << "起点必须设置在非障碍物处！" << '\n';
        init_start_point_ = false;
        return false;
    }

    // printf("起点设置成功，位置：(%d, %d)\n", x_grid, y_grid);
    start_node_ = &map_[y_grid][x_grid];
    init_start_point_ = true;
    return true;
}

bool Astar::setStartPoint(const cv::Point2d & p)
{
    return setStartPoint(p.x, p.y);
}

bool Astar::setEndPoint(const double x, const double y)
{
    if (init_map_info_ == false)
    {
        std::cout << "设置终点前需设置地图信息！" << '\n';
        init_end_point_ = false;
        return false;
    }

    int x_grid = static_cast<int>((x - ori_x_) / res_);
    int y_grid = static_cast<int>((y - ori_y_) / res_);

    if ((x_grid >= 0 && x_grid < cols_ && y_grid >= 0 && y_grid < rows_) == false)
    {
        std::cout << "终点必须设置在地图内部！\n";
        init_end_point_ = false;
        return false;
    }

    if (map_[y_grid][x_grid].cost >= params_.map.OBSTACLE_THRESHOLD)
    {
        std::cout << "终点必须设置在非障碍物处！\n";
        init_end_point_ = false;
        return false;
    }

    // printf("终点设置成功，位置：(%d, %d)\n", x_grid, y_grid);
    end_node_ = &map_[y_grid][x_grid];
    init_end_point_ = true;
    return true;
}

bool Astar::setEndPoint(const cv::Point2d & p)
{
    return setEndPoint(p.x, p.y);
}

bool Astar::getProcessedMap(cv::Mat & map) const
{
    if (!init_map_)
    {
        std::cout << "当前无有效地图！\n";
        return false;
    }
    
    map = cv::Mat::zeros(rows_, cols_, CV_8UC1);
    for (size_t i = 0; i < rows_; i++)
    {
        for (size_t j = 0; j < cols_; j++)
        {
            map.at<uchar>(i, j) = map_[i][j].cost;
        }
    }
    return true;
}

bool Astar::getPath(std::vector<cv::Point2d> & path, std::vector<std::vector<cv::Point2d>> & auxiliary_info)
{
    if (!(init_map_ && init_start_point_ && init_end_point_ && init_map_info_))
    {
        std::cout << "请先设置地图、起点和终点！\n";
        return false;
    }
    
    // 初始节点
    helper_.resetResultInfo();
    auto start_time = std::chrono::steady_clock::now();
    start_node_->g = 0;
    start_node_->h = getH(start_node_->point);
    start_node_->f = start_node_->g + start_node_->h;
    start_node_->parent_node = nullptr;
    std::priority_queue<Node*, std::vector<Node *>, Node::NodePointerCmp> queue;
    queue.push(start_node_);


    // 循环遍历
    while (queue.empty() == false)          // 队列为空，一般意味着无法到达终点
    {
        Node * const this_node = queue.top();
        const cv::Point2i & this_point = this_node->point;
        queue.pop();

        this_node->type = Node::NodeType::CLOSED;
        if (this_node == end_node_)  // 已经到达终点
            break;
        
        // 八个方向扩展搜索，即3x3方格内部
        for (int k = -1; k <= 1; k++)
        {
            for (int l = -1; l <= 1; l++)
            {
                if (((k != 0 || l != 0) &&                                          // 排除自身节点
                     (this_point.y + k >= 0 && this_point.y + k < rows_) &&
                     (this_point.x + l >= 0 && this_point.x + l < cols_)) == false) // 保证当前索引不溢出
                {
                    continue;
                }
                
                Node * const new_node = &map_[this_point.y + k][this_point.x + l];
                const cv::Point2i & new_point = new_node->point;

                if (new_node->cost < params_.map.OBSTACLE_THRESHOLD &&   // 保证该节点是非障碍物节点，才能联通。代价地图[0, 100] 0可通过 100不可通过 
                    new_node->type != Node::NodeType::CLOSED)                   // 不对已加入CLOSED的数据再次判断
                {
                    const double dis = std::hypot(new_point.x - this_point.x, new_point.y - this_point.y);

                    if (new_node->type == Node::NodeType::UNKNOWN)
                    {
                        new_node->g = this_node->g + dis;
                        new_node->h = getH(new_point);
                        new_node->f = new_node->g + new_node->h;
                        new_node->parent_node = this_node;
                        new_node->type = Node::NodeType::OPENED;
                        queue.push(std::move(new_node));

                        // 按顺序记录访问到的节点
                        helper_.search.nodes.push_back(cv::Point2d((new_point.x + 0.5) * res_ + ori_x_,
                                                                          (new_point.y + 0.5) * res_ + ori_y_));
                    }
                    else //new_node->type == NodeType::OPENED
                    {
                        if (new_node->g > this_node->g + dis)   // 更新最近距离
                        {
                            new_node->g = this_node->g + dis;
                            new_node->f = new_node->g + new_node->h;
                            new_node->parent_node = this_node;
                        }
                    }

                    helper_.search.node_counter++;       // 访问节点的总次数
                }
            }
        }    
    }


    // 保存结果路径
    path.clear();
    auxiliary_info.clear();
    const Node * path_node = end_node_;
    while (path_node != nullptr)
    {
        path.push_back(cv::Point2d((path_node->point.x + 0.5) * res_ + ori_x_,
                                   (path_node->point.y + 0.5) * res_ + ori_y_));
        path_node = path_node->parent_node;
    }
    std::reverse(path.begin(), path.end());

    // 保存结果信息
    auto end_time = std::chrono::steady_clock::now();
    helper_.search.node_nums = helper_.search.nodes.size();           // 访问到的节点数
    helper_.search.path_length = path.size();                                // 路径长度
    helper_.search.cost_time = (end_time - start_time).count() / 1000000.0;  // 算法耗时 ms
    auxiliary_info.push_back(helper_.search.nodes);

    // 复原地图
    for (int i = 0; i < rows_; i++)
    {
        for (int j = 0; j < cols_; j++)
        {
            map_[i][j].g = 0;
            map_[i][j].h = 0;
            map_[i][j].f = 0;
            map_[i][j].type = Node::NodeType::UNKNOWN;
            map_[i][j].parent_node = nullptr;
        }
    }
    
    return path.size() > 1;
}


double Astar::getH(const cv::Point2i & p) const
{
    double h = 0;
    double dx = 0.0;
    double dy = 0.0;
    switch (params_.cost_function.HEURISTICS_TYPE)
    {
    case HeuristicsType::None:
        break;
    case HeuristicsType::Manhattan:
        h = std::fabs(p.x - end_node_->point.x) + std::fabs(p.y - end_node_->point.y);
        break;
    case HeuristicsType::Euclidean:
        h = std::sqrt(std::pow(p.x - end_node_->point.x, 2) + std::pow(p.y - end_node_->point.y, 2));
        break;
    case HeuristicsType::Chebyshev:
        h = std::max(std::abs(p.x - end_node_->point.x), std::abs(p.y - end_node_->point.y));
        break;
    case HeuristicsType::Octile:
        static constexpr double k = 0.4142135623730950; // std::sqrt(2) - 1
        dx = std::fabs(p.x - end_node_->point.x);
        dy = std::fabs(p.y - end_node_->point.y);
        h = std::max(dx, dy) + k * std::min(dx, dy);
        break;
    default:
        break;
    }
    return h;
}
// ========================= Astar =========================
