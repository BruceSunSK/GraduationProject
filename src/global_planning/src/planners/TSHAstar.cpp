#include "global_planning/planners/TSHAstar.h"


// ========================= TSHAstar::TSHAstarHelper =========================

std::string TSHAstar::TSHAstarHelper::paramsInfo() const
{
    const TSHAstar * TSHAstar_planner = dynamic_cast<const TSHAstar *>(planner_);
    const TSHAstarParams & TSHAstar_params = TSHAstar_planner->params_;

    std::stringstream params_info;
    params_info << "[Params Info]:\n";
    PRINT_STRUCT(params_info, TSHAstar_params);
    return params_info.str();
}

std::string TSHAstar::TSHAstarHelper::mapInfo() const
{
    const TSHAstar * TSHAstar_planner = dynamic_cast<const TSHAstar *>(planner_);

    std::stringstream map_info;
    map_info << "[Map Info]:\n"
             << "  rows: "        << TSHAstar_planner->rows_ << std::endl
             << "  cols: "        << TSHAstar_planner->cols_ << std::endl
             << "  resolution: "  << TSHAstar_planner->res_ << std::endl
             << "  start point: " << TSHAstar_planner->start_point_ << std::endl
             << "  end point:   " << TSHAstar_planner->end_point_ << std::endl;
    return map_info.str();
}

std::string TSHAstar::TSHAstarHelper::resultInfo() const
{
    std::stringstream result_info;
    result_info << "[Result Info]:\n";
    PRINT_STRUCT(result_info, search);
    return result_info.str();
}
// ========================= TSHAstar::TSHAstarHelper =========================


// ========================= TSHAstar::Node =========================

const int TSHAstar::Node::neighbor_offset_table[8][2] =
    {
        {-1,  0},
        { 1,  0},
        { 0, -1},    
        { 0,  1},
        {-1, -1},
        {-1,  1},
        { 1, -1},
        { 1,  1}
    };

const TSHAstar::Node::Direction TSHAstar::Node::direction_table[8] =
    {
        Direction::N,
        Direction::S,
        Direction::W,
        Direction::E,
        Direction::NW,
        Direction::NE,
        Direction::SW,
        Direction::SE
    };
// ========================= TSHAstar::Node =========================


// ========================= TSHAstar =========================

REGISTER_ENUM_BODY(NeighborType,
                   REGISTER_MEMBER(NeighborType::FourConnected),
                   REGISTER_MEMBER(NeighborType::FiveConnected),
                   REGISTER_MEMBER(NeighborType::EightConnected));

REGISTER_ENUM_BODY(HeuristicsType,
                   REGISTER_MEMBER(HeuristicsType::None),
                   REGISTER_MEMBER(HeuristicsType::Manhattan),
                   REGISTER_MEMBER(HeuristicsType::Euclidean),
                   REGISTER_MEMBER(HeuristicsType::Chebyshev),
                   REGISTER_MEMBER(HeuristicsType::Octile));

REGISTER_ENUM_BODY(PathSimplificationType,
                   REGISTER_MEMBER(PathSimplificationType::DouglasPeucker),
                   REGISTER_MEMBER(PathSimplificationType::DistanceThreshold),
                   REGISTER_MEMBER(PathSimplificationType::AngleThreshold),
                   REGISTER_MEMBER(PathSimplificationType::DPPlus));

REGISTER_ENUM_BODY(PathSmoothType,
                   REGISTER_MEMBER(PathSmoothType::Bezier),
                   REGISTER_MEMBER(PathSmoothType::BSpline));

void TSHAstar::initParams(const GlobalPlannerParams & params)
{
    const TSHAstarParams & p = dynamic_cast<const TSHAstarParams &>(params);
    params_ = p;
}

bool TSHAstar::setMap(const cv::Mat & map)
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
            if (cost >= params_.map.EXPANDED_MIN_THRESHOLD &&
                cost <= params_.map.EXPANDED_MAX_THRESHOLD) // 只对正常范围[0, 100]中的值进行膨胀扩展；(100, 255)未定义区域和255为探索区域不进行扩展。可手动修改
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
                                                            * cost * params_.map.EXPANDED_K);
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
    // 0. 设置搜索时所用的特例地图，并过滤掉小代价值栅格
    search_map_.clear();
    search_map_.resize(rows_);
    for (int i = 0; i < rows_; i++)
    {
        std::vector<Node> row(cols_);
        for (int j = 0; j < cols_; j++)
        {
            Node node;
            node.point.x = j;
            node.point.y = i;
            if (out.at<uchar>(i, j) >= params_.map.COST_THRESHOLD)    // 过滤小代价值栅格
            {
                node.cost = out.at<uchar>(i, j);
            }
            else
            {
                node.cost = 0;
                out.at<uchar>(i, j) = 0;
            }

            row[j] = std::move(node);
        }
        search_map_[i] = std::move(row);
    }
    // 1. 离散代价值的障碍物地图，正常范围在[0, 100]，也包括255未探明的情况
    discrete_map_ = std::move(out);
    // 2. 代价值二值化后的障碍物地图，未探明情况视为障碍物。即[0, OBSTACLE_THRESHOLD)值为0，[OBSTACLE_THRESHOLD, 255]值为255
    cv::threshold(discrete_map_, binary_map_, params_.map.OBSTACLE_THRESHOLD - 1, 255, cv::ThresholdTypes::THRESH_BINARY);
    // 3. 在二值化地图中进行distanceTransform变换的结果，用于描述每个栅格到障碍物的距离
    cv::distanceTransform(binary_map_, distance_map_, cv::DIST_L2, cv::DIST_MASK_PRECISE);

    // printf("地图设置成功，大小 %d x %d\n", rows_, cols_);
    init_map_ = true;
    return true;
}

bool TSHAstar::setStartPoint(const double x, const double y)
{
    if (init_map_info_ == false)
    {
        std::cout << "设置起点前需设置地图信息！" << '\n';
        init_start_point_ = false;
        return false;
    }

    double x_grid_double = (x - ori_x_) / res_;
    double y_grid_double = (y - ori_y_) / res_;
    int x_grid = static_cast<int>(x_grid_double);
    int y_grid = static_cast<int>(y_grid_double);

    if ((x_grid >= 0 && x_grid < cols_ && y_grid >= 0 && y_grid < rows_) == false)
    {
        std::cout << "起点必须设置在地图内部！" << '\n';
        init_start_point_ = false;
        return false;
    }

    if (search_map_[y_grid][x_grid].cost >= params_.map.OBSTACLE_THRESHOLD)
    {
        std::cout << "起点必须设置在非障碍物处！" << '\n';
        init_start_point_ = false;
        return false;
    }

    // printf("起点设置成功，位置：(%d, %d)\n", x_grid, y_grid);
    start_point_.x = x_grid_double;
    start_point_.y = y_grid_double;
    start_node_ = &search_map_[y_grid][x_grid];
    init_start_point_ = true;
    return true;
}

bool TSHAstar::setStartPoint(const cv::Point2d & p)
{
    return setStartPoint(p.x, p.y);
}

bool TSHAstar::setStartPointYaw(const double yaw)
{
    if (!init_start_point_)
    {
        std::cout << "设置起点朝向前必须先设置起点！\n";
        return false;
    }

    // 确定起点的朝向
    if (yaw > -M_PI / 8 && yaw <= M_PI / 8)
    {
        start_node_->direction_to_parent = Node::Direction::E;
    }
    else if (yaw > M_PI / 8 && yaw <= 3 * M_PI / 8)
    {
        start_node_->direction_to_parent = Node::Direction::SE;
    }
    else if (yaw > 3 * M_PI / 8 && yaw <= 5 * M_PI / 8)
    {
        start_node_->direction_to_parent = Node::Direction::S;
    }
    else if (yaw > 5 * M_PI / 8 && yaw <= 7 * M_PI / 8)
    {
        start_node_->direction_to_parent = Node::Direction::SW;
    }
    else if (yaw > 7 * M_PI / 8 || yaw <= -7 * M_PI / 8)
    {
        start_node_->direction_to_parent = Node::Direction::W;
    }
    else if (yaw > -7 * M_PI / 8 && yaw <= -5 * M_PI / 8)
    {
        start_node_->direction_to_parent = Node::Direction::NW;
    }
    else if (yaw > -5 * M_PI / 8 && yaw <= -3 * M_PI / 8)
    {
        start_node_->direction_to_parent = Node::Direction::N;
    }
    else if (yaw > -3 * M_PI / 8 && yaw <= -M_PI / 8)
    {
        start_node_->direction_to_parent = Node::Direction::NE;
    }
    else
    {
        std::cout << "设置起点朝向失败！\n";
        return false;
    }
    return true;
}

bool TSHAstar::setEndPoint(const double x, const double y)
{
    if (init_map_info_ == false)
    {
        std::cout << "设置终点前需设置地图信息！" << '\n';
        init_end_point_ = false;
        return false;
    }

    double x_grid_double = (x - ori_x_) / res_;
    double y_grid_double = (y - ori_y_) / res_;
    int x_grid = static_cast<int>(x_grid_double);
    int y_grid = static_cast<int>(y_grid_double);

    if ((x_grid >= 0 && x_grid < cols_ && y_grid >= 0 && y_grid < rows_) == false)
    {
        std::cout << "终点必须设置在地图内部！\n";
        init_end_point_ = false;
        return false;
    }

    if (search_map_[y_grid][x_grid].cost >= params_.map.OBSTACLE_THRESHOLD)
    {
        std::cout << "终点必须设置在非障碍物处！\n";
        init_end_point_ = false;
        return false;
    }

    // printf("终点设置成功，位置：(%d, %d)\n", x_grid, y_grid);
    end_point_.x = x_grid_double;
    end_point_.y = y_grid_double;
    end_node_ = &search_map_[y_grid][x_grid];
    init_end_point_ = true;
    return true;
}

bool TSHAstar::setEndPoint(const cv::Point2d & p)
{
    return setEndPoint(p.x, p.y);
}

bool TSHAstar::setEndPointYaw(const double yaw)
{
}

bool TSHAstar::getProcessedMap(cv::Mat & map) const
{
    if (!init_map_)
    {
        std::cout << "当前无有效地图！\n";
        return false;
    }
    
    map = discrete_map_.clone();
    return true;
}

bool TSHAstar::getPath(std::vector<cv::Point2d> & path, std::vector<std::vector<cv::Point2d>> & auxiliary_info)
{
    if (!(init_map_ && init_start_point_ && init_end_point_ && init_map_info_))
    {
        std::cout << "请先设置地图、起点和终点！\n";
        return false;
    }

    // 0.初始化helper
    helper_.resetResultInfo();

    // 第一大步：搜索过程建立原始参考路径，并进行初步平滑
    // 1.搜索方法规划原始路径
    std::vector<Node *> raw_nodes;
    if (!generateRawNodes(raw_nodes))
    {
        std::cout << "规划失败！目标点不可达！\n";
        resetSearchMap();
        return false;
    }

    // 2.节点转成路径点，复原搜索用的地图
    std::vector<cv::Point2i> raw_path;
    nodesToPath(raw_nodes, raw_path);

    // 3.去除冗余节点
    std::vector<cv::Point2i> reduced_path;
    removeRedundantPoints(raw_path, reduced_path);

    // 4.完成路径平滑
    std::vector<cv::Point2d> smooth_path;
    smoothPath(reduced_path, smooth_path);

    // 5.转换成ReferencePath格式，并进行优化平滑
    Path::ReferencePath::Ptr reference_path;
    if (!optimizePath(smooth_path, reference_path))
    {
        std::cout << "路径优化平滑失败！\n";
        return false;
    }


    path.clear();
    path = reference_path->GetPath();
    std::for_each(path.begin(), path.end(), [this](cv::Point2d & p)
        {
            p.x = p.x * res_ + ori_x_;
            p.y = p.y * res_ + ori_y_;
        });

    
    // 第二大步：使用采样开辟解空间，利用Frenet坐标系优化路径


    // 辅助信息赋值
    // 7.auxiliary_info赋值
    auxiliary_info.clear();
    auxiliary_info.push_back(std::move(helper_.search.path_search.raw_path));
    auxiliary_info.push_back(std::move(helper_.search.path_search.nodes));
    auxiliary_info.push_back(std::move(helper_.search.path_simplification.reduced_path));
    auxiliary_info.push_back(std::move(helper_.search.path_smooth.smooth_path));
    auxiliary_info.push_back(std::move(helper_.search.path_optimization.sample_path));
    auxiliary_info.push_back(std::move(helper_.search.path_optimization.optimized_path));

    return true;
}


std::vector<size_t> TSHAstar::getNeighborsIndex(const Node * const node) const
{
    std::vector<size_t> index_range;
    switch (params_.search.path_search.NEIGHBOR_TYPE)
    {
    case NeighborType::FourConnected:
        index_range = { 0, 1, 2, 3 };
        break;
    case NeighborType::FiveConnected:
        switch (node->direction_to_parent)
        {
        case Node::Direction::N:
            index_range = { 0, 2, 3, 4, 5 };
            break;
        case Node::Direction::S:
            index_range = { 1, 2, 3, 6, 7 };
            break;
        case Node::Direction::W:
            index_range = { 0, 1, 2, 4, 6 };
            break;
        case Node::Direction::E:
            index_range = { 0, 1, 3, 5, 7 };
            break;
        case Node::Direction::NW:
            index_range = { 0, 2, 4, 5, 6 };
            break;
        case Node::Direction::NE:
            index_range = { 0, 3, 4, 5, 7 };
            break;
        case Node::Direction::SW:
            index_range = { 1, 2, 4, 6, 7 };
            break;
        case Node::Direction::SE:
            index_range = { 1, 3, 5, 6, 7 };
            break;
        default:    // UNKNOWN情形下，返回全部邻居
            index_range = { 0, 1, 2, 3, 4, 5, 6, 7 };
            break;
        }
        break;
    case NeighborType::EightConnected:
        index_range = { 0, 1, 2, 3, 4, 5, 6, 7 };
        break;
    default:
        break;
    }
    return index_range;
}

double TSHAstar::getG(const Node * const n, const Node::Direction direction, const Node::Direction par_direction) const
{
    uint8_t dir = static_cast<uint8_t>(direction);
    uint8_t par_dir = static_cast<uint8_t>(par_direction);

    const double dis_cost = (dir & 0x10) == 0x00 ? 1 : 1.4142135623730950;                          // 扩展该节点的距离代价
    const double trav_cost = std::exp(n->cost * 0.01 * params_.search.path_search.TRAV_COST_K);   // 可通行度代价系数
    double turn_cost = 1;
    // 不可能出现反向直行的情况，因为parent的parent一定已经加入了CLOSE集合，在搜索parent的扩展节点时一定不会有parent的parent
    if (dir == par_dir)                         // 同向直行
    {
        turn_cost = params_.search.path_search.TURN_COST_STRAIGHT;
    }
    else if ((dir & par_dir) == dir ||
             (dir & par_dir) == par_dir)        // 同向斜行
    {
        turn_cost = params_.search.path_search.TURN_COST_SLANT;
    }
    else if (((dir ^ par_dir) & 0x10) == 0x00)  // 垂向直行
    {
        turn_cost = params_.search.path_search.TURN_COST_VERTICAL;
    }
    else                                        // 反向斜行
    {
        turn_cost = params_.search.path_search.TURN_COST_REVERSE_SLANT;
    }
    return trav_cost * turn_cost * dis_cost;
}

void TSHAstar::getH(Node * const n) const
{
    cv::Point2i & p = n->point;
    double h = 0;
    double dx = 0.0;
    double dy = 0.0;
    switch (params_.search.path_search.HEURISTICS_TYPE)
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

void TSHAstar::getW(Node * const n) const
{
    // n->w = 1;
    // return;

    if (n->w >= 0) // 有数据说明已经存在针对当前地图的计算结果，直接返回
    {
        return;
    }

    // 计算周围节点的平均代价
    const int kernel_size = 11;
    int num = 0;
    double sum = 0.0;
    for (int k = -kernel_size / 2; k <= kernel_size / 2; k++)
    {
        for (int l = -kernel_size / 2; l <= kernel_size / 2; l++)   
        {
            if ((n->point.y + k >= 0 && n->point.y + k < rows_) &&
                (n->point.x + l >= 0 && n->point.x + l < cols_)) // 保证当前索引不溢出
            {
                num++;
                sum += search_map_[n->point.y + k][n->point.x + l].cost;
            }
        }
    }
    // 障碍物密度 P ∈ (0, 1)
    const double P = sum / num / 100.0;
    // 权重 w ∈ (1, ∞)
    n->w = 1 - 1 * std::log(P);
}

void TSHAstar::resetSearchMap()
{
    for (int i = 0; i < rows_; i++)
    {
        for (int j = 0; j < cols_; j++)
        {
            search_map_[i][j].g = 0;
            search_map_[i][j].h = 0;
            search_map_[i][j].w = 1;
            search_map_[i][j].f = 0;
            search_map_[i][j].type = Node::NodeType::UNKNOWN;
            search_map_[i][j].parent_node = nullptr;
            search_map_[i][j].direction_to_parent = Node::Direction::UNKNOWN;
        }
    }
}

bool TSHAstar::generateRawNodes(std::vector<Node *> & raw_nodes)
{
    // 初始节点
    auto start_time = std::chrono::steady_clock::now();
    start_node_->g = 0;
    getH(start_node_);
    // getW(start_node_);
    // start_node_->f = start_node_->g + start_node_->w * start_node_->h;
    start_node_->f = start_node_->g + start_node_->h;
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
        
        // 根据参数，确定搜素领域的范围
        std::vector<size_t> index_range = getNeighborsIndex(this_node);
        for (size_t & index : index_range)
        {
            const int & k = Node::neighbor_offset_table[index][0];
            const int & l = Node::neighbor_offset_table[index][1];
            if (((this_point.y + k >= 0 && this_point.y + k < rows_) &&
                 (this_point.x + l >= 0 && this_point.x + l < cols_)) == false) // 保证当前索引不溢出
            {
                continue;
            }
            
            Node * const new_node = &search_map_[this_point.y + k][this_point.x + l];
            const cv::Point2i & new_point = new_node->point;
            if ((new_node->cost < params_.map.OBSTACLE_THRESHOLD &&         // 保证该节点是非障碍物节点，才能联通。代价地图[0, 100] 0可通过 100不可通过 
                    new_node->type != Node::NodeType::CLOSED) == false)     // 不对已加入CLOSED的数据再次判断
            {
                continue;
            }

            const Node::Direction direction = Node::direction_table[index];
            const double gi = getG(new_node, direction, this_node->direction_to_parent);
            if (new_node->type == Node::NodeType::UNKNOWN)
            {
                new_node->g = this_node->g + gi;
                getH(new_node);
                // getW(new_node);
                // new_node->f = new_node->g + new_node->w * new_node->h;
                new_node->f = new_node->g + new_node->h;
                new_node->type = Node::NodeType::OPENED;
                new_node->parent_node = this_node;
                new_node->direction_to_parent = direction;
                queue.push(std::move(new_node));

                // 按顺序记录访问到的节点
                helper_.search.path_search.nodes.push_back(cv::Point2d((new_point.x + 0.5) * res_ + ori_x_,
                                                                  (new_point.y + 0.5) * res_ + ori_y_));

            }
            else //new_node->type == NodeType::OPENED
            {
                if (new_node->g > this_node->g + gi)        // 更新最近距离
                {
                    new_node->g = this_node->g + gi;
                    // new_node->f = new_node->g + new_node->w * new_node->h;
                    new_node->f = new_node->g + new_node->h;
                    new_node->parent_node = this_node;
                    new_node->direction_to_parent = direction;
                }
            }

            helper_.search.path_search.raw_node_counter++;       // 访问节点的总次数
        }
    }


    // 保存结果路径
    raw_nodes.clear();
    Node * path_node = end_node_;
    while (path_node != nullptr)
    {
        raw_nodes.push_back(path_node);
        helper_.search.path_search.raw_path.push_back(cv::Point2d((path_node->point.x + 0.5) * res_ + ori_x_,
                                                             (path_node->point.y + 0.5) * res_ + ori_y_));

        path_node = path_node->parent_node;
    }
    std::reverse(raw_nodes.begin(), raw_nodes.end());                                           // 翻转使得路径点从起点到终点排列
    std::reverse(helper_.search.path_search.raw_path.begin(), helper_.search.path_search.raw_path.end()); // 翻转使得路径点从起点到终点排列

    // 保存结果信息
    auto end_time = std::chrono::steady_clock::now();
    helper_.search.path_search.raw_node_nums = helper_.search.path_search.nodes.size();       // 访问到的节点数
    helper_.search.path_search.raw_path_size = helper_.search.path_search.raw_path.size();  // 路径长度
    helper_.search.path_search.cost_time = (end_time - start_time).count() / 1000000.0;  // 算法耗时 ms
    return raw_nodes.size() > 1;
}

void TSHAstar::nodesToPath(const std::vector<Node *> & nodes, std::vector<cv::Point2i> & path)
{
    path.clear();
    for (const Node * const n : nodes)
    {
        path.push_back(n->point);
    }

    resetSearchMap();
}

void TSHAstar::removeRedundantPoints(const std::vector<cv::Point2i> & raw_path, std::vector<cv::Point2i> & reduced_path)
{
    reduced_path.clear();
    if (raw_path.size() <= 2)  // 只有两个点，说明只是起点和终点
    {
        reduced_path.insert(reduced_path.end(), raw_path.begin(), raw_path.end());
        helper_.search.path_simplification.reduced_path_size = reduced_path.size();
        helper_.search.path_simplification.cost_time = 0;
        return;
    }

    auto start_time = std::chrono::steady_clock::now();
    switch (params_.search.path_simplification.PATH_SIMPLIFICATION_TYPE)
    {
    // 1.使用Douglas-Peucker法去除冗余点
    // 测试结果：× 1. 也会破坏对称结构。
    //         √ 2. 但保留的点都很具有特征点代表性
    //         √ 3. 感觉在徐工地图上超级棒，甚至感觉不用平滑，用原始的折线也不错。
    case PathSimplificationType::DouglasPeucker:
        Path::Simplification::DouglasPeucker(raw_path, reduced_path, params_.search.path_simplification.DISTANCE_THRESHOLD / res_);
        break;

    // 2.使用垂距限值法去除冗余点
    // 测试结果：√ 1. 能够较为有效的保留对称的形状；
    //         × 2. 但是只去除一轮的话，（由于节点特别密集）会存在连续的2 / 3个点都存在，还需要二次去除
    case PathSimplificationType::DistanceThreshold:
        Path::Simplification::DistanceThreshold(raw_path, reduced_path, params_.search.path_simplification.DISTANCE_THRESHOLD / res_);
        break;

    // 3.使用角度限值法去除冗余点
    // 测试结果：× 1. 感觉容易破坏对称结构。本来对称的节点去除冗余点后有较为明显的差异，导致效果变差。
    //         √ 2. 在长直线路径下，能够有效去除多余歪点，只剩起点终点。
    case PathSimplificationType::AngleThreshold:
        Path::Simplification::AngleThreshold(raw_path, reduced_path, params_.search.path_simplification.ANGLE_THRESHOLD);
        break;

    // 4.使用改进的Douglas-Peucker法去除冗余点
    // 测试结果：在DP的基础上，增加了对障碍物的处理。
    case PathSimplificationType::DPPlus:
        Path::Simplification::DPPlus(binary_map_, raw_path, reduced_path, params_.search.path_simplification.OBSTACLE_THRESHOLD,
                                                                          params_.search.path_simplification.DISTANCE_THRESHOLD / res_, 
                                                         static_cast<int>(params_.search.path_simplification.LINE_WIDTH / res_ + 0.5),
                                                                          params_.search.path_simplification.MAX_INTAVAL / res_);
        break;
        
    default:
        reduced_path = raw_path;
        break;
    }
    auto end_time = std::chrono::steady_clock::now();

    helper_.search.path_simplification.reduced_path_size = reduced_path.size();
    helper_.search.path_simplification.cost_time = (end_time - start_time).count() / 1000000.0;
    std::for_each(reduced_path.begin(), reduced_path.end(), [this](cv::Point2i & p)
        {
            this->helper_.search.path_simplification.reduced_path.push_back(cv::Point2d((p.x + 0.5) * res_ + ori_x_,
                                                                                        (p.y + 0.5) * res_ + ori_y_));
        });
}

bool TSHAstar::smoothPath(const std::vector<cv::Point2i> & raw_path, std::vector<cv::Point2d> & smooth_path)
{
    smooth_path.clear();
    if (raw_path.size() == 0)
    {
        return false;
    }

    auto start_time = std::chrono::steady_clock::now();
    switch (params_.search.path_smooth.PATH_SMOOTH_TYPE)
    {
    // 使用优化后的分段三阶贝塞尔曲线进行平滑
    // 测试结果：不好。
    //         1. 手动拼接处曲率变化较为明显。
    //         2. 曲线会更倾向于中间，而非控制点。在两个控制点间隔较远时会更加明显，效果更差。
    //         3. 速度比B样条慢。毕竟B样条我狠狠优化过，贝塞尔貌似只能搞dp计算组合数打表。
    case PathSmoothType::Bezier:
        Curve::BezierCurve::PiecewiseSmoothCurve(raw_path, smooth_path, params_.search.path_smooth.T_STEP);
        break;

    // 使用B样条曲线进行平滑
    // 测试结果：不错。
    //         1. 效果很好，曲线平滑，作用在全局，控制点均匀分布。
    //         2. 速度快。
    case PathSmoothType::BSpline:
        Curve::BSplineCurve::SmoothCurve(raw_path, smooth_path, 4, params_.search.path_smooth.T_STEP);
        break;

    default:
        break;
    }
    auto end_time = std::chrono::steady_clock::now();

    helper_.search.path_smooth.smooth_path_size = smooth_path.size();
    helper_.search.path_smooth.cost_time = (end_time - start_time).count() / 1000000.0;
    // 保存info信息时，和之前类似，全部正确的转换到真实坐标系下。
    // 但从此处开始，后续的工作全部都不需要栅格整数点，而是使用离散点，因此在此处消除掉所有0.5的偏移，但分辨率不变，只有在最后输出时才乘以分辨率。
    std::for_each(smooth_path.begin(), smooth_path.end(), [this](cv::Point2d & p)
        {
            p.x += 0.5;
            p.y += 0.5;
            this->helper_.search.path_smooth.smooth_path.push_back(cv::Point2d(p.x * res_ + ori_x_,
                                                                               p.y * res_ + ori_y_));
        });
    
    return true;
}

bool TSHAstar::optimizePath(const std::vector<cv::Point2d> & path, Path::ReferencePath::Ptr & reference_path)
{
    auto start_time = std::chrono::steady_clock::now();

    // 将原始离散点转换成参考线数据类型，并进行均匀采样
    Path::ReferencePath::Ptr raw_reference_path(new Path::ReferencePath(path, params_.search.path_optimization.S_INTERVAL));

    // 使用离散点对参考线通过数值优化进行平滑。
    // 代价函数包括：1.平滑代价。 2.长度代价。 3.偏离代价
    // 约束：无
    const std::array<double, 3> weights = { params_.search.path_optimization.REF_WEIGTH_SMOOTH,
                                            params_.search.path_optimization.REF_WEIGTH_LENGTH,
                                            params_.search.path_optimization.REF_WEIGTH_DEVIATION};
    Smoother::DiscretePointSmoother smoother(weights, params_.search.path_optimization.REF_BUFFER_DISTANCE);
    if (!smoother.Solve(raw_reference_path, reference_path))
    {
        return false;
    }
    auto end_time = std::chrono::steady_clock::now();

    helper_.search.path_optimization.cost_time = (end_time - start_time).count() / 1000000.0;
    helper_.search.path_optimization.sample_path = raw_reference_path->GetPath();
    helper_.search.path_optimization.sample_path_size = helper_.search.path_optimization.sample_path.size();
    helper_.search.path_optimization.sample_path_length = raw_reference_path->GetLength();
    helper_.search.path_optimization.optimized_path = reference_path->GetPath();
    helper_.search.path_optimization.optimized_path_size = helper_.search.path_optimization.optimized_path.size();
    helper_.search.path_optimization.optimized_path_length = reference_path->GetLength();
    std::for_each(helper_.search.path_optimization.sample_path.begin(),
                  helper_.search.path_optimization.sample_path.end(), 
                  [this](cv::Point2d & p)
        {
            p.x = p.x * res_ + ori_x_;
            p.y = p.y * res_ + ori_y_;
        });
    std::for_each(helper_.search.path_optimization.optimized_path.begin(),
                  helper_.search.path_optimization.optimized_path.end(), 
                  [this](cv::Point2d & p)
        {
            p.x = p.x * res_ + ori_x_;
            p.y = p.y * res_ + ori_y_;
        });
    
    return true;
}
// ========================= TSHAstar =========================
