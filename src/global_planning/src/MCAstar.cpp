#include "global_planning/MCAstar.h"


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


/// @brief 对规划器相关变量进行初始化设置，进行参数拷贝设置
/// @param params 传入的参数
void MCAstar::initParams(const GlobalPlannerParams & params)
{
    const MCAstarParams & p = dynamic_cast<const MCAstarParams &>(params);
    params_ = p;
}

/// @brief 对输入的map的进行预处理膨胀、过滤小代价值栅格，然后设置成为规划器中需要使用的地图
/// @param map 输入的原始地图
/// @return 地图设置是否成功
bool MCAstar::setMap(const cv::Mat & map)
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


    // 地图预处理膨胀
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
    const cv::Mat & src = map;
    cv::Mat out = map.clone();
    const int kernel_half_width = kernel.cols / 2;
    for (int i = 0; i < rows_; i++)
    {
        for (int j = 0; j < cols_; j++)
        {
            const uchar & cost = src.at<uchar>(i, j);
            if (cost >= params_.map_params.EXPANDED_MIN_THRESHOLD &&
                cost <= params_.map_params.EXPANDED_MAX_THRESHOLD) // 只对正常范围[0, 100]中的值进行膨胀扩展；(100, 255)未定义区域和255为探索区域不进行扩展。可手动修改
            {            
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
                        
                        uchar new_cost = static_cast<uchar>(kernel.at<double>(m + kernel_half_width, n + kernel_half_width) 
                                                            * cost * params_.map_params.EXPANDED_K);
                        new_cost = std::min<uchar>(new_cost, 100);  // 防止超过100范围               
                        if (out.at<uchar>(i + m, j + n) < new_cost)
                        {
                            out.at<uchar>(i + m, j + n) = new_cost;
                        }
                    }
                }
            }
        }
    }


    // 设置地图
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
            if (out.at<uchar>(i, j) >= params_.map_params.COST_THRESHOLD)    // 过滤小代价值栅格
            {
                node.cost = out.at<uchar>(i, j);   
            }
            else
            {
                node.cost = 0;
            }
            
            row[j] = std::move(node);
        }
        map_[i] = std::move(row);
    }

    printf("地图设置成功，大小 %d x %d\n", rows_, cols_);

    init_map_ = true;
    return true;
}

/// @brief 设置规划路径的起点。以栅格坐标形式，而非行列形式。
/// @param x 栅格坐标系的x值
/// @param y 栅格坐标系的y值
/// @return 该点是否能够成为起点。即该点在地图内部且不在障碍物上。
bool MCAstar::setStartPoint(const int x, const int y)
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

/// @brief 设置规划路径的起点。以栅格坐标形式，而非行列形式。
/// @param p 栅格坐标系的点
/// @return 该点是否能够成为起点。即该点在地图内部且不在障碍物上。
bool MCAstar::setStartPoint(const cv::Point2i p)
{
    return setStartPoint(p.x, p.y);
}

/// @brief 设置规划路径的终点。以栅格坐标形式，而非行列形式。
/// @param x 栅格坐标系的x值
/// @param y 栅格坐标系的y值
/// @return 该点是否能够成为终点。即该点在地图内部且不在障碍物上。
bool MCAstar::setEndPoint(const int x, const int y)
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

/// @brief 设置规划路径的终点。以栅格坐标形式，而非行列形式。
/// @param p 栅格坐标系的点
/// @return 该点是否能够成为终点。即该点在地图内部且不在障碍物上。
bool MCAstar::setEndPoint(const cv::Point2i p)
{
    return setEndPoint(p.x, p.y);
}

/// @brief 获得处理后的地图，即算法内部真正使用的，经过膨胀、忽视小代价值后的地图
/// @param map 地图将存入该变量
/// @return 存入是否成功
bool MCAstar::getProcessedMap(cv::Mat & map)
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

    // 2.节点转成路径点
    std::vector<cv::Point2i> raw_path;
    nodesToPath(raw_nodes, raw_path);
    // nodesToPath(raw_nodes, path);

    ////////
    removeRedundantPoints(raw_path, path);
    ////////

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

    // 2.节点转成路径点
    std::vector<cv::Point2i> raw_path;
    nodesToPath(raw_nodes, raw_path);

    // 3.去除冗余节点
    std::vector<cv::Point2i> reduced_path;
    removeRedundantPoints(raw_path, reduced_path);

    // 4.完成路径平滑
    smoothPath(reduced_path, path);

    // 5.降采样
    downsampling(path);

    // 6.复原地图
    resetMap();

    return true;
}


/// @brief 根据启发类型，得到节点n到终点的启发值
/// @param n 待计算的节点，计算该节点到终点的启发值
void MCAstar::getH(Node * const n)
{
    cv::Point2i & p = n->point;
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
    n->h = h;
}

/// @brief 计算p点到终点的启发权重
/// @param p 待计算的点，计算该点到终点的启发权重
void MCAstar::getW(Node * const n)
{
    n->w = 1;
    return;

    // 暂时不好用，以下全部不用
    const int min_y = std::min(n->point.y, end_node_->point.y);
    const int max_y = std::max(n->point.y, end_node_->point.y);
    const int min_x = std::min(n->point.x, end_node_->point.x);
    const int max_x = std::max(n->point.x, end_node_->point.x);
    
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
                    const double dis_with_trav_cost = dis * std::exp(new_node->cost * 0.01 * params_.cost_function_params.TRAV_COST_K);

                    if (new_node->type == Node::NodeType::UNKNOWN)
                    {
                        new_node->g = this_node->g + dis_with_trav_cost;
                        getH(new_node);
                        getW(new_node);
                        new_node->f = new_node->g + new_node->w * new_node->h;
                        new_node->type = Node::NodeType::OPENED;
                        new_node->parent_node = this_node;
                        new_node->direction_to_parent = Node::index_direction_map.at({k, l});
                        queue.push(std::move(new_node));
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

/// @brief 将节点Node数据类型转换成便于后续计算的Point数据类型
/// @param nodes 输入的待转换节点
/// @param path 输出的路径
void MCAstar::nodesToPath(const std::vector<Node *> & nodes, std::vector<cv::Point2i> & path)
{
    path.clear();
    for (const Node * const n : nodes)
    {
        path.push_back(n->point);
    }
}

/// @brief 根据路径节点之间的关系，去除中间的冗余点，只保留关键点，如：起始点、转弯点
/// @param raw_path 输入的待去除冗余点的路径
/// @param reduced_path 输出的去除冗余点后的路径
void MCAstar::removeRedundantPoints(const std::vector<cv::Point2i> & raw_path, std::vector<cv::Point2i> & reduced_path)
{
    reduced_path.clear();
    if (raw_path.size() <= 2)  // 只有两个点，说明只是起点和终点
    {
        reduced_path.insert(reduced_path.end(), raw_path.begin(), raw_path.end());
        return;
    }

    switch (params_.path_simplification_params.PATH_SIMPLIFICATION_TYPE)
    {
    // 1.使用Douglas-Peucker法去除冗余点
    // 测试结果：× 1. 也会破坏对称结构。
    //         √ 2. 但保留的点都很具有特征点代表性
    //         √ 3. 感觉在徐工地图上超级棒，甚至感觉不用平滑，用原始的折现也不错。
    case PathSimplificationType::DouglasPeucker:
        PathSimplification::Douglas_Peucker(raw_path, reduced_path, params_.path_simplification_params.THRESHOLD / res_);
        break;

    // 2.使用垂距限值法去除冗余点
    // 测试结果：√ 1. 能够较为有效的保留对称的形状；
    //         × 2. 但是只去除一轮的话，（由于节点特别密集）会存在连续的2 / 3个点都存在，还需要二次去除
    case PathSimplificationType::DistanceThreshold:
        PathSimplification::distance_threshold(raw_path, reduced_path, params_.path_simplification_params.THRESHOLD / res_);
        break;

    // 3.使用角度限值法去除冗余点
    // 测试结果：× 1. 感觉容易破坏对称结构。本来对称的节点去除冗余点后有较为明显的差异，导致效果变差。
    //         √ 2. 在长直线路径下，能够有效去除多余歪点，只剩起点终点。
    case PathSimplificationType::AngleThreshold:
        PathSimplification::angle_threshold(raw_path, reduced_path, params_.path_simplification_params.THRESHOLD);
        break;

    default:
        reduced_path = raw_path;
        break;
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
    const size_t points_num = raw_path.size();
    if (points_num == 0)
    {
        return false;
    }
    
    // 使用优化后的分段三阶贝塞尔曲线进行平滑
    BezierCurve::piecewise_smooth_curve(raw_path, smooth_path, params_.bezier_curve_params.T_STEP);

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
void MCAstar::downsampling(std::vector<cv::Point2d> & path)
{
    const size_t size = path.size();
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
        if (temp_dis > params_.downsampling_params.INTERVAL)
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
            map_[i][j].w = 1;
            map_[i][j].w_cost = 0;
            map_[i][j].f = 0;
            map_[i][j].type = Node::NodeType::UNKNOWN;
            map_[i][j].parent_node = nullptr;
            map_[i][j].direction_to_parent = Node::Direction::UNKNOWN;
        }
    }
}
