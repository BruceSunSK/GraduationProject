#include "global_planning/MC_astar.h"

const std::unordered_map<std::pair<int, int>, 
                         MCAstar::Node::Direction, 
                         MCAstar::Node::HashPair, 
                         MCAstar::Node::EqualPair> MCAstar::Node::index_direction_map = 
    {
        {{-1,  0}, Direction::N},
        {{ 1,  0}, Direction::S},
        {{ 0, -1}, Direction::W},
        {{ 0,  1}, Direction::E},
        {{-1, -1}, Direction::NW},
        {{-1,  1}, Direction::NE},
        {{ 1, -1}, Direction::SW},
        {{ 1,  1}, Direction::SE}
    };


MCAstar::MCAstar(HeuristicsType type) : type_(type)
{
}

MCAstar::~MCAstar()
{
}

bool MCAstar::setMap(const cv::Mat & map)
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
            node.cost = map.at<uchar>(i, j);
            row[j] = node;
        }
        map_[i] = row;
    }

    printf("地图设置成功，大小 %d x %d\n", rows_, cols_);

    init_map_ = true;
    return true;
}

bool MCAstar::setStartPoint(const int x, const int y)
{
    if ((x >= 0 && x < cols_ && y >= 0 && y < rows_) == false)
    {
        std::cout << "起点必须设置在地图内部！" << '\n';
        init_start_node_ = false;
        return false;
    }

    if (map_[y][x].cost >= 100)
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

bool MCAstar::setStartPoint(const cv::Point2i p)
{
    return setStartPoint(p.x, p.y);
}

bool MCAstar::setEndPoint(const int x, const int y)
{
    if ((x >= 0 && x < cols_ && y >= 0 && y < rows_) == false)
    {
        std::cout << "终点必须设置在地图内部！\n";
        init_end_node_ = false;
        return false;
    }

    if (map_[y][x].cost >= 100)
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

bool MCAstar::setEndPoint(const cv::Point2i p)
{
    return setEndPoint(p.x, p.y);
}

/// @brief 通过给定的地图、起点、终点规划出一条从起点到终点的路径。
/// @param path 规划出的路径。该路径是栅格坐标系下原始路径点，没有冗余点剔除、平滑、降采样操作。
/// @return 是否规划成功
bool MCAstar::getRawPath(std::vector<cv::Point2i> & path)
{
    if ( !(init_map_ && init_start_node_ && init_end_node_) )
    {
        std::cout << "请先设置地图、起点和终点！\n";
        return false;
    }

    // 1.规划原始路径
    std::vector<Node *> raw_nodes;
    if (!generateRawNodes(raw_nodes))
    {
        std::cout << "规划失败！目标点不可达！\n";
        return false;
    }

    ////////
    std::vector<Node *> reduced_nodes;
    removeRedundantNodes(raw_nodes, reduced_nodes);
    ////////

    // 2.节点转成路径点
    nodesToPath(reduced_nodes, path);

    // 3.复原地图
    resetMap();

    return true;
}

/// @brief 通过给定的地图、地图信息、起点、终点规划出一条从起点到终点的平滑路径。
/// @param path 规划出的路径。该路径是真实地图下的坐标点，进行冗余点剔除、分段三阶贝塞尔曲线平滑、降采样操作。并且补齐0.5个单位长度的栅格偏差。
/// @return 是否规划成功
bool MCAstar::getSmoothPath(std::vector<cv::Point2d> & path)
{
    if ( !(init_map_ && init_start_node_ && init_end_node_) )
    {
        std::cout << "请先设置地图、起点和终点！\n";
        return false;
    }

    // 1.规划原始路径
    std::vector<Node *> raw_nodes;
    if (!generateRawNodes(raw_nodes))
    {
        std::cout << "规划失败！目标点不可达！\n";
        return false;
    }
    
    // 2.去除冗余节点
    std::vector<Node *> reduced_nodes;
    removeRedundantNodes(raw_nodes, reduced_nodes);

    // 3.节点转成路径点
    std::vector<cv::Point2i> reduced_path;
    nodesToPath(reduced_nodes, reduced_path);

    // 4.完成路径平滑
    smoothPath(reduced_path, path);

    // 5.降采样
    downsampling(path, res_);

    // 6.复原地图
    resetMap();

    return true;
}


/// @brief 根据启发类型，得到节点n到终点的启发值
/// @param n 待计算的节点，计算该节点到终点的启发值
void MCAstar::getH(Node * n)
{
    cv::Point2i & p = n->point;
    double h = 0;
    double dx = 0.0;
    double dy = 0.0;
    switch (type_)
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
    n->h = h;
}

/// @brief 计算p点到终点的启发权重
/// @param p 待计算的点，计算该点到终点的启发权重
void MCAstar::getW(Node * n)
{
    n->w = 1;
    return;

    // 暂时不好用，以下全部不用
    int min_y = std::min(n->point.y, end_node_->point.y);
    int max_y = std::max(n->point.y, end_node_->point.y);
    int min_x = std::min(n->point.x, end_node_->point.x);
    int max_x = std::max(n->point.x, end_node_->point.x);
    
    n->w_cost = 0;
    for (int i = min_y; i <= max_y; i++)
    {
        for (int j = min_x; j <= max_x; j++)
        {
            n->w_cost += map_[i][j].cost;
        }
    }

    // 可能是一种优化的方法，todo
    // // 针对起点，需要首次计算全部代价值
    // if (n->parent_node == nullptr)
    // {

    //     for (int i = min_y; i <= max_y; i++)
    //     {
    //         for (int j = min_x; j <= max_x; j++)
    //         {
    //             n->w_cost += map_[i][j].cost;
    //         }
    //     }
    // }
    // // 其余点可以简化计算，利用父节点处计算出的代价值减去扩展的值
    // else
    // {
    //     if (n->point.y == n->parent_node->point.y)   // 同一行
    //     {
    //         // false 表明需要减去代价值；true表示需要新增代价值
    //         bool add_flag = (n->point.x > n->parent_node->point.x) ^ (end_node_->point.x > n->parent_node->point.x); 

    //         if (add_flag)
    //         {
    //             for (int i = min_y; i <= max_y; i++)
    //             {
    //                 n->w_cost += map_[i][n->point.x].cost;
    //             }
    //         }
    //         else
    //         {
    //             for (int i = min_y; i <= max_y; i++)
    //             {
    //                 n->w_cost -= map_[i][n->parent_node->point.x].cost;
    //             }
    //         }
    //     }

    //     if (n->point.x == n->parent_node->point.x)  // 同一列
    //     {
    //         // false 表明需要减去代价值；true表示需要新增代价值
    //         bool add_flag = (n->point.y > n->parent_node->point.y) ^ (end_node_->point.y > n->parent_node->point.y); 

    //         if (add_flag)
    //         {
    //             for (int j = min_x; j <= max_x; j++)
    //             {
    //                 n->w_cost += map_[n->point.y][j].cost;
    //             }
    //         }
    //         else
    //         {
    //             for (int j = min_x; j <= max_x; j++)
    //             {
    //                 n->w_cost -= map_[n->parent_node->point.y][j].cost;
    //             }
    //         }
    //     }
    // }
    
    // 障碍物密度 P ∈ (0, 1)
    const double P = n->w_cost * 0.01 / ((std::abs(n->point.x - end_node_->point.x) + 1) * (std::abs(n->point.y - end_node_->point.y) + 1));
    // 权重 w ∈ (1, ∞)
    n->w = 1 - std::log(P);
}

/// @brief 通过代价地图计算得到的原始路径，未经过去除冗余点、平滑操作
/// @param raw_nodes 输出规划的原始结果
/// @return 规划是否成功。如果起点和终点不可达则规划失败
bool MCAstar::generateRawNodes(std::vector<Node *> & raw_nodes)
{
    // 初始节点
    start_node_->g = 0;
    getH(start_node_);
    getW(start_node_);
    start_node_->f = start_node_->g + start_node_->w * start_node_->h;
    start_node_->parent_node = nullptr;
    std::priority_queue<Node *, std::vector<Node*>, Node::NodePointerCmp> queue;
    queue.push(start_node_);


    // 循环遍历
    while (queue.empty() == false)  // 队列为空，一般意味着无法到达终点
    {
        Node * this_node = queue.top();
        cv::Point2i this_point = this_node->point;
        queue.pop();

        this_node->type = Node::NodeType::CLOSED;
        if (this_node == end_node_)  // 已经到达终点
            break;
        
        // 八个方向扩展搜索，即3x3方格内部
        for (int k = -1; k <= 1; k++)
        {
            for (int l = -1; l <= 1; l++)
            {
                if ((k != 0 || l != 0) &&   // 排除自身节点
                    (this_point.y+k >= 0 && this_point.y+k < rows_) &&
                    (this_point.x+l >= 0 && this_point.x+l < cols_)) // 保证当前索引不溢出
                {
                    Node * new_node = &map_[this_point.y+k][this_point.x+l];
                    cv::Point2i new_point = new_node->point;

                    if (new_node->cost < 100 &&  // 保证该节点是非障碍物节点，才能联通。代价地图[0, 100] 0可通过 100不可通过 
                        new_node->type != Node::NodeType::CLOSED) // 不对已加入CLOSED的数据再次判断
                    {
                        double dis = std::sqrt(std::pow(new_point.x - this_point.x, 2) + std::pow(new_point.y - this_point.y, 2));
                        double dis_with_trav_cost = dis * std::exp(new_node->cost * 0.02);
   
                        if (new_node->type == Node::NodeType::UNKNOWN)
                        {
                            new_node->g = this_node->g + dis_with_trav_cost;
                            getH(new_node);
                            getW(new_node);
                            new_node->f = new_node->g + new_node->w * new_node->h;
                            new_node->type = Node::NodeType::OPENED;
                            new_node->parent_node = this_node;
                            new_node->direction_to_parent = Node::index_direction_map.at({k, l});
                            queue.push(new_node);
                        }
                        else //new_node->type == NodeType::OPENED
                        {
                            if (new_node->g > this_node->g + dis_with_trav_cost)   // 更新最近距离
                            {
                                new_node->g = this_node->g + dis_with_trav_cost;
                                new_node->f = new_node->g + new_node->w * new_node->h;
                                new_node->parent_node = this_node;
                                new_node->direction_to_parent = Node::index_direction_map.at({k, l});
                            }
                        }
                    }
                }
            }
        }    
    }


    // 保存结果路径
    raw_nodes.clear();
    Node * path_node = end_node_;
    while (path_node != nullptr)
    {
        raw_nodes.push_back(path_node);
        path_node = path_node->parent_node;
    }
    std::reverse(raw_nodes.begin(), raw_nodes.end()); // 翻转使得路径点从起点到终点排列
    return raw_nodes.size() > 1;
}

/// @brief 根据路径节点之间的关系，去除中间的冗余点，只保留关键点，如：起始点、转弯点
/// @param raw_nodes 输入的待去除冗余点的路径
/// @param reduced_nodes 输出的去除冗余点后的路径
/// @return 是否去除冗余点成功。若没有去除任何点，则返回false
bool MCAstar::removeRedundantNodes(const std::vector<Node *> & raw_nodes, std::vector<Node *> & reduced_nodes)
{
    reduced_nodes.clear();
    size_t size = raw_nodes.size();
    if (size <= 2)  // 只有两个点，说明只是起点和终点
    {
        reduced_nodes.insert(reduced_nodes.end(), raw_nodes.begin(), raw_nodes.end());
        return false;
    }
    
    // 如果连续两个点扩展的方向一致，则去除第一个点，只保留第二个点
    reduced_nodes.push_back(raw_nodes.front());
    size_t l = 1;
    for (size_t r = 2; r < size - 1; r++)
    {
        if (raw_nodes[r]->direction_to_parent != raw_nodes[l]->direction_to_parent)
        {
            reduced_nodes.push_back(raw_nodes[r - 1]);
            l = r;
        }
    }
    if (raw_nodes[size - 1]->direction_to_parent != raw_nodes[size - 2]->direction_to_parent) // 最后两个点单独判断
    {
        reduced_nodes.push_back(raw_nodes[size - 2]);
    }
    reduced_nodes.push_back(raw_nodes.back());
    return true;
}

/// @brief 将节点Node数据类型转换成便于后续计算的Point数据类型
/// @param nodes 输入的待转换节点
/// @param path 输出的路径
void MCAstar::nodesToPath(const std::vector<Node *> & nodes, std::vector<cv::Point2i> & path)
{
    path.clear();
    for (Node * n : nodes)
    {
        path.push_back(n->point);
    }
}

/// @brief 使用分段三阶贝塞尔曲线进行平滑。该函数将栅格整数坐标xy的值进行平滑，得到平滑路径后再消除栅格偏差的0.5个单位长度
/// @param raw_path 待平滑路径
/// @param soomth_path 输出平滑后的路径
/// @param control_nums_per_subpath 每个子路径的点的数目
/// @return 平滑操作是否成功。若缺少地图信息或输入路径为空，返回false
bool MCAstar::smoothPath(const std::vector<cv::Point2i> & raw_path, std::vector<cv::Point2d> & smooth_path)
{
    if (init_map_info_ == false)
    {
        std::cout << "平滑失败！缺少地图信息用以补齐栅格偏差！\n";
        return false;
    }

    smooth_path.clear();
    size_t points_num = raw_path.size();
    if (points_num == 0)
    {
        return false;
    }
    
    // 使用优化后的分段三阶贝塞尔曲线进行平滑
    bezier_.piecewise_smooth_curve(raw_path, smooth_path);

    // 消除0.5个栅格偏差
    std::for_each(smooth_path.begin(), smooth_path.end(), [this](cv::Point2d & p){
            p.x = (p.x + 0.5) * res_ + ori_x_;
            p.y = (p.y + 0.5) * res_ + ori_y_;
        });
    
    return true;
}

/// @brief 对路径进行降采样。在进行完贝塞尔曲线平滑后进行，避免规划出的路径过于稠密
/// @param path 待降采样的路径
/// @param dis 两点间最小间距
void MCAstar::downsampling(std::vector<cv::Point2d> & path, double dis)
{
    size_t size = path.size();
    if (size == 0)
    {
        return;
    }
    
    std::vector<cv::Point2d> temp;
    temp.push_back(path.front());   // 保证起点
    cv::Point2d temp_p = path.front();
    for (size_t i = 1; i < size - 1; i++)
    {
        double temp_dis = std::hypot(path[i].x - temp_p.x, path[i].y - temp_p.y);
        if (temp_dis > dis)
        {
            temp.push_back(path[i]);
            temp_p = path[i];
        }
    }
    temp.push_back(path.back());    // 保证终点
    path = temp;
}

/// @brief 将当前地图中的参数全部初始化。一般在完成一次规划的所有步骤后进行。
void MCAstar::resetMap()
{
    for (int i = 0; i < rows_; i++)
    {
        for (int j = 0; j < cols_; j++)
        {
            map_[i][j].g = 0;
            map_[i][j].h = 0;
            map_[i][j].w = 0;
            map_[i][j].w_cost = 0;
            map_[i][j].f = 0;
            map_[i][j].type = Node::NodeType::UNKNOWN;
            map_[i][j].parent_node = nullptr;
            map_[i][j].direction_to_parent = Node::Direction::UNKNOWN;
        }
    }
}
