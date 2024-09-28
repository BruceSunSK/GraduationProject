#include "global_planning/astar.h"


/// @brief 对规划器相关变量进行初始化设置，进行参数拷贝设置
/// @param params 传入的参数
void Astar::initParams(const GlobalPlannerParams & params)
{
    const AstarParams & p = dynamic_cast<const AstarParams &>(params);
    params_ = p;
}

double Astar::getH(const cv::Point2i & p)
{
    double h = 0;
    double dx = 0.0;
    double dy = 0.0;
    switch (params_.cost_function_params.HEURISTICS_TYPE)
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
            node.cost = (map.at<uchar>(i, j) >= params_.map_params.OBSTACLE_THRESHOLD &&
                         map.at<uchar>(i, j) != 255 ? 100 : 0);
            node.type = Node::NodeType::UNKNOWN;
            row[j] = std::move(node);
        }
        map_[i] = std::move(row);
    }

    printf("地图设置成功，大小 %d x %d\n", rows_, cols_);

    init_map_ = true;
    return true;
}

bool Astar::setStartPoint(const int x, const int y)
{
    if ((x >= 0 && x < cols_ && y >= 0 && y < rows_) == false)
    {
        std::cout << "起点必须设置在地图内部！" << '\n';
        init_start_node_ = false;
        return false;
    }

    if (map_[y][x].cost >= params_.map_params.OBSTACLE_THRESHOLD)
    {
        std::cout << "起点必须设置在非障碍物处！" << '\n';
        init_start_node_ = false;
        return false;
    }

    printf("起点设置成功，位置：(%d, %d)\n", x, y);
    
    start_node_ = &map_[y][x];
    init_start_node_ = true;
    return true;
}

bool Astar::setStartPoint(const cv::Point2i p)
{
    return setStartPoint(p.x, p.y);
}

bool Astar::setEndPoint(const int x, const int y)
{
    if ((x >= 0 && x < cols_ && y >= 0 && y < rows_) == false)
    {
        std::cout << "终点必须设置在地图内部！\n";
        init_end_node_ = false;
        return false;
    }

    if (map_[y][x].cost >= params_.map_params.OBSTACLE_THRESHOLD)
    {
        std::cout << "终点必须设置在非障碍物处！\n";
        init_end_node_ = false;
        return false;
    }

    printf("终点设置成功，位置：(%d, %d)\n", x, y);

    end_node_ = &map_[y][x];
    init_end_node_ = true;
    return true;
}

bool Astar::setEndPoint(const cv::Point2i p)
{
    return setEndPoint(p.x, p.y);
}

bool Astar::getProcessedMap(cv::Mat & map)
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

bool Astar::getRawPath(std::vector<cv::Point2i> & path)
{
    if ( !(init_map_ && init_start_node_ && init_end_node_) )
    {
        std::cout << "请先设置地图、起点和终点！\n";
        return false;
    }
    
    
    // 初始节点
    start_node_->g = 0;
    start_node_->h = getH(start_node_->point);
    start_node_->f = start_node_->g + start_node_->h;
    start_node_->parent_node = nullptr;
    std::priority_queue<Node*, std::vector<Node *>, Node::NodePointerCmp> queue;
    queue.push(start_node_);


    // 循环遍历
    while (queue.empty() == false)  // 队列为空，一般意味着无法到达终点
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

                if (new_node->cost < params_.map_params.OBSTACLE_THRESHOLD &&   // 保证该节点是非障碍物节点，才能联通。代价地图[0, 100] 0可通过 100不可通过 
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
                }
            }
        }    
    }


    // 保存结果路径
    path.clear();
    const Node * path_node = end_node_;
    while (path_node != nullptr)
    {
        path.push_back(path_node->point);
        path_node = path_node->parent_node;
    }
    std::reverse(path.begin(), path.end());

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

bool Astar::getSmoothPath(std::vector<cv::Point2d> & path)
{
    if (init_map_info_ == false)
    {
        std::cout << "平滑失败！缺少地图信息用以补齐栅格偏差！\n";
        return false;
    }
    
    path.clear();
    std::vector<cv::Point2i> temp;
    if (!getRawPath(temp))
    {
        return false;
    }

    // 消除0.5个栅格偏差
    std::for_each(temp.begin(), temp.end(), [this, &path](cv::Point2i & p){
            cv::Point2d pf;
            pf.x = (p.x + 0.5) * res_ + ori_x_;
            pf.y = (p.y + 0.5) * res_ + ori_y_;
            path.push_back(pf);
        });

    return true;
}
