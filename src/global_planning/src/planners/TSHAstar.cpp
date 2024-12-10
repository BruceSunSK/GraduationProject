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
    PRINT_STRUCT(result_info, sample);
    return result_info.str();
}
// ========================= TSHAstar::TSHAstarHelper =========================


// ========================= TSHAstar::SearchNode =========================

const int TSHAstar::SearchNode::neighbor_offset_table[8][2] =
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

const TSHAstar::SearchNode::Direction TSHAstar::SearchNode::direction_table[8] =
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
// ========================= TSHAstar::SearchNode =========================


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
        std::vector<SearchNode> row(cols_);
        for (int j = 0; j < cols_; j++)
        {
            SearchNode node;
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
    // 2. 代价值二值化后的障碍物地图，未探明情况视为障碍物。即[0, OBSTACLE_THRESHOLD)值为255，白色，前景；[OBSTACLE_THRESHOLD, 255]值为0，黑色，背景。
    cv::threshold(discrete_map_, binary_map_, params_.map.OBSTACLE_THRESHOLD - 1, 255, cv::ThresholdTypes::THRESH_BINARY_INV);
    // 3. 在二值化地图中进行distanceTransform变换的结果，用于描述每个栅格到障碍物的距离
    cv::distanceTransform(binary_map_, distance_map_, cv::DIST_L2, cv::DIST_MASK_PRECISE, CV_32FC1);
    // 4. 设置采样时所用的地图
    sample_map_.SetMap(distance_map_);


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

    start_yaw_ = Math::NormalizeAngle(yaw);

    // 确定起点的朝向
    if (start_yaw_ > -M_PI / 8 && start_yaw_ <= M_PI / 8)
    {
        start_node_->direction_to_parent = SearchNode::Direction::E;
    }
    else if (start_yaw_ > M_PI / 8 && start_yaw_ <= 3 * M_PI / 8)
    {
        start_node_->direction_to_parent = SearchNode::Direction::SE;
    }
    else if (start_yaw_ > 3 * M_PI / 8 && start_yaw_ <= 5 * M_PI / 8)
    {
        start_node_->direction_to_parent = SearchNode::Direction::S;
    }
    else if (start_yaw_ > 5 * M_PI / 8 && start_yaw_ <= 7 * M_PI / 8)
    {
        start_node_->direction_to_parent = SearchNode::Direction::SW;
    }
    else if (start_yaw_ > 7 * M_PI / 8 || start_yaw_ <= -7 * M_PI / 8)
    {
        start_node_->direction_to_parent = SearchNode::Direction::W;
    }
    else if (start_yaw_ > -7 * M_PI / 8 && start_yaw_ <= -5 * M_PI / 8)
    {
        start_node_->direction_to_parent = SearchNode::Direction::NW;
    }
    else if (start_yaw_ > -5 * M_PI / 8 && start_yaw_ <= -3 * M_PI / 8)
    {
        start_node_->direction_to_parent = SearchNode::Direction::N;
    }
    else if (start_yaw_ > -3 * M_PI / 8 && start_yaw_ <= -M_PI / 8)
    {
        start_node_->direction_to_parent = SearchNode::Direction::NE;
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
    if (!init_end_point_)
    {
        std::cout << "设置终点朝向前必须先设置终点！\n";
        return false;
    }
    end_yaw_ = Math::NormalizeAngle(yaw);
    return true;
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
    std::vector<SearchNode *> raw_nodes;
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
    if (!optimizeDiscretePointsPath(smooth_path, reference_path))
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

    
    

    path.clear();
    path = reference_path->GetPath();
    std::for_each(path.begin(), path.end(), [this](cv::Point2d & p)
        {
            p.x = p.x * res_ + ori_x_;
            p.y = p.y * res_ + ori_y_;
        });

    
    // 第二大步：使用采样开辟解空间，利用Frenet坐标系优化路径
    // 6.使用dp动态规划开辟凸空间，找到粗解，确定可行域范围
    std::vector<std::pair<double, double>> bounds;
    if (!findPathTunnel(reference_path, bounds))
    {
        std::cout << "路径DP失败！\n";
        return false;
    }

    // 7. 使用分段加加速度进行平滑，得到最终路径
    if (!optimizePiecewiseJerkPath(reference_path, bounds, path))
    {
        std::cout << "分段加加速度平滑失败！\n";
        return false;
    }

    
    // 辅助信息赋值
    // 7.auxiliary_info赋值
    auxiliary_info.clear();
    auxiliary_info.push_back(std::move(helper_.search.path_search.raw_path));
    auxiliary_info.push_back(std::move(helper_.search.path_search.nodes));
    auxiliary_info.push_back(std::move(helper_.search.path_simplification.reduced_path));
    auxiliary_info.push_back(std::move(helper_.search.path_smooth.smooth_path));
    auxiliary_info.push_back(std::move(helper_.search.path_optimization.sample_path));
    auxiliary_info.push_back(std::move(helper_.search.path_optimization.optimized_path));
    auxiliary_info.push_back(std::move(helper_.sample.path_dp.dp_path));
    auxiliary_info.push_back(std::move(helper_.sample.path_dp.lower_bound));
    auxiliary_info.push_back(std::move(helper_.sample.path_dp.upper_bound));

    return true;
}


std::vector<size_t> TSHAstar::getNeighborsIndex(const SearchNode * const node) const
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
        case SearchNode::Direction::N:
            index_range = { 0, 2, 3, 4, 5 };
            break;
        case SearchNode::Direction::S:
            index_range = { 1, 2, 3, 6, 7 };
            break;
        case SearchNode::Direction::W:
            index_range = { 0, 1, 2, 4, 6 };
            break;
        case SearchNode::Direction::E:
            index_range = { 0, 1, 3, 5, 7 };
            break;
        case SearchNode::Direction::NW:
            index_range = { 0, 2, 4, 5, 6 };
            break;
        case SearchNode::Direction::NE:
            index_range = { 0, 3, 4, 5, 7 };
            break;
        case SearchNode::Direction::SW:
            index_range = { 1, 2, 4, 6, 7 };
            break;
        case SearchNode::Direction::SE:
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

double TSHAstar::getG(const SearchNode * const n,
                      const SearchNode::Direction direction,
                      const SearchNode::Direction par_direction) const
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

void TSHAstar::getH(SearchNode * const n) const
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

void TSHAstar::getW(SearchNode * const n) const
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
            search_map_[i][j].type = SearchNode::NodeType::UNKNOWN;
            search_map_[i][j].parent_node = nullptr;
            search_map_[i][j].direction_to_parent = SearchNode::Direction::UNKNOWN;
        }
    }
}

bool TSHAstar::generateRawNodes(std::vector<SearchNode *> & raw_nodes)
{
    // 初始节点
    auto start_time = std::chrono::steady_clock::now();
    start_node_->g = 0;
    getH(start_node_);
    // getW(start_node_);
    // start_node_->f = start_node_->g + start_node_->w * start_node_->h;
    start_node_->f = start_node_->g + start_node_->h;
    start_node_->parent_node = nullptr;
    std::priority_queue<SearchNode *, std::vector<SearchNode*>, SearchNode::NodePointerCmp> queue;
    queue.push(start_node_);


    // 循环遍历
    while (queue.empty() == false)  // 队列为空，一般意味着无法到达终点
    {
        SearchNode * const this_node = queue.top();
        const cv::Point2i & this_point = this_node->point;
        queue.pop();

        this_node->type = SearchNode::NodeType::CLOSED;
        if (this_node == end_node_)  // 已经到达终点
            break;
        
        // 根据参数，确定搜素领域的范围
        std::vector<size_t> index_range = getNeighborsIndex(this_node);
        for (size_t & index : index_range)
        {
            const int & k = SearchNode::neighbor_offset_table[index][0];
            const int & l = SearchNode::neighbor_offset_table[index][1];
            if (((this_point.y + k >= 0 && this_point.y + k < rows_) &&
                 (this_point.x + l >= 0 && this_point.x + l < cols_)) == false) // 保证当前索引不溢出
            {
                continue;
            }
            
            SearchNode * const new_node = &search_map_[this_point.y + k][this_point.x + l];
            const cv::Point2i & new_point = new_node->point;
            if ((new_node->cost < params_.map.OBSTACLE_THRESHOLD &&         // 保证该节点是非障碍物节点，才能联通。代价地图[0, 100] 0可通过 100不可通过 
                 new_node->type != SearchNode::NodeType::CLOSED) == false)  // 不对已加入CLOSED的数据再次判断
            {
                continue;
            }

            const SearchNode::Direction direction = SearchNode::direction_table[index];
            const double gi = getG(new_node, direction, this_node->direction_to_parent);
            if (new_node->type == SearchNode::NodeType::UNKNOWN)
            {
                new_node->g = this_node->g + gi;
                getH(new_node);
                // getW(new_node);
                // new_node->f = new_node->g + new_node->w * new_node->h;
                new_node->f = new_node->g + new_node->h;
                new_node->type = SearchNode::NodeType::OPENED;
                new_node->parent_node = this_node;
                new_node->direction_to_parent = direction;
                queue.push(std::move(new_node));

                // 按顺序记录访问到的节点
                helper_.search.path_search.nodes.emplace_back((new_point.x + 0.5) * res_ + ori_x_,
                                                              (new_point.y + 0.5) * res_ + ori_y_);

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
    SearchNode * path_node = end_node_;
    while (path_node != nullptr)
    {
        raw_nodes.push_back(path_node);
        helper_.search.path_search.raw_path.emplace_back((path_node->point.x + 0.5) * res_ + ori_x_,
                                                         (path_node->point.y + 0.5) * res_ + ori_y_);

        path_node = path_node->parent_node;
    }
    std::reverse(raw_nodes.begin(), raw_nodes.end());                                                       // 翻转使得路径点从起点到终点排列
    std::reverse(helper_.search.path_search.raw_path.begin(), helper_.search.path_search.raw_path.end());   // 翻转使得路径点从起点到终点排列

    // 保存结果信息
    auto end_time = std::chrono::steady_clock::now();
    helper_.search.path_search.raw_node_nums = helper_.search.path_search.nodes.size();     // 访问到的节点数
    helper_.search.path_search.raw_path_size = helper_.search.path_search.raw_path.size();  // 路径长度
    helper_.search.path_search.cost_time = (end_time - start_time).count() / 1000000.0;     // 算法耗时 ms
    return raw_nodes.size() > 1;
}

void TSHAstar::nodesToPath(const std::vector<SearchNode *> & nodes, std::vector<cv::Point2i> & path)
{
    path.clear();
    for (const SearchNode * const n : nodes)
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
            this->helper_.search.path_simplification.reduced_path.emplace_back((p.x + 0.5) * res_ + ori_x_,
                                                                               (p.y + 0.5) * res_ + ori_y_);
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
            this->helper_.search.path_smooth.smooth_path.emplace_back(p.x * res_ + ori_x_, p.y * res_ + ori_y_);
        });
    
    return true;
}

bool TSHAstar::optimizeDiscretePointsPath(const std::vector<cv::Point2d> & path, Path::ReferencePath::Ptr & reference_path)
{
    auto start_time = std::chrono::steady_clock::now();

    // 将原始离散点转换成参考线数据类型，并进行均匀采样
    Path::ReferencePath::Ptr raw_reference_path =
        std::make_shared<Path::ReferencePath>(path, params_.search.path_optimization.S_INTERVAL / res_);

    // 使用离散点对参考线通过数值优化进行平滑。
    const std::array<double, 3> weights = { params_.search.path_optimization.REF_WEIGTH_SMOOTH,
                                            params_.search.path_optimization.REF_WEIGTH_LENGTH,
                                            params_.search.path_optimization.REF_WEIGTH_DEVIATION};
    Smoother::DiscretePointSmoother smoother(weights, params_.search.path_optimization.REF_BUFFER_DISTANCE / res_);
    if (!smoother.Solve(raw_reference_path, reference_path))
    {
        return false;
    }
    auto end_time = std::chrono::steady_clock::now();

    // 保存辅助信息
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

void TSHAstar::calCostInSampleNodes(std::vector<std::vector<SampleNode>> & sample_nodes, const size_t lon_idx, const size_t lat_idx)
{
    if (lon_idx == 0 || !sample_nodes[lon_idx][lat_idx].is_feasible)
    {
        return;
    }

    const static double lateral_sample_range   = params_.sample.path_sample.LATERAL_SAMPLE_RANGE / res_;
    const static double dp_warning_distance    = params_.sample.path_dp.COLLISION_DISTANCE / res_;
    const static double dp_weight_offset       = params_.sample.path_dp.WEIGHT_OFFSET;
    const static double dp_weight_obstacle     = params_.sample.path_dp.WEIGHT_OBSTACLE;
    const static double dp_weight_angle_change = params_.sample.path_dp.WEIGHT_ANGLE_CHANGE;
    const static double dp_weight_angle_diff   = params_.sample.path_dp.WEIGHT_ANGLE_DIFF;

    SampleNode & curr_node = sample_nodes[lon_idx][lat_idx];
    double self_cost = 0.0;

    // 1. 偏离代价。根据当前点偏离对应参考点的程度算出。
    self_cost += (dp_weight_offset * std::abs(curr_node.l) / lateral_sample_range);
    // 2. 障碍物代价。根据当前点距离最近不可碰撞障碍物的距离算出。
    if (curr_node.dis_to_obs < dp_warning_distance)     // 过近的碰撞范围已经在采样时设置为不可通过，此处只需要判断是否较近即可
    {
        self_cost += (dp_weight_obstacle * Math::Lerp(1.0, 0.0, curr_node.dis_to_obs / dp_warning_distance));
    }
    // 该采样点与上一层所有采样点依次连接比较，使用dp找出最优路径。
    double min_cost = std::numeric_limits<double>::max();
    for (SampleNode & prev_node : sample_nodes[lon_idx - 1])
    {
        if (!prev_node.is_feasible)     // 跳过上一个不可通过的点
        {
            continue;
        }
        if (std::abs(curr_node.l - prev_node.l) > 4 * (curr_node.s - prev_node.s))  // 角度变化过大，则跳过
        {
            continue;
        }

        // 前向欧拉法确定该采样点处的朝向角度
        const double heading = std::atan2(curr_node.y - prev_node.y, curr_node.x - prev_node.x);
        // 3. 角度变化代价。根据当前点与上一层采样点的角度差异算出。转换[0, 1]的系数再乘以权重。
        const double angle_change_cost = (dp_weight_angle_change * std::abs(Math::NormalizeAngle(heading - prev_node.heading)) / M_PI_2);
        // 4. 角度差异代价。根据当前点与参考点的角度差异算出。转换[0, 1]的系数再乘以权重。
        const double angle_diff_cost = (dp_weight_angle_diff * std::abs(Math::NormalizeAngle(heading - curr_node.theta)) / M_PI_2);

        // 使用dp判断代价
        const double curr_cost = self_cost + angle_change_cost + angle_diff_cost;
        const double total_cost = curr_cost + prev_node.cost;
        if (total_cost < min_cost)
        {
            min_cost = total_cost;
            curr_node.heading = heading;
            curr_node.parent = &prev_node;
        }
    }
    // 记录最小代价路径。
    curr_node.cost = min_cost;
}

bool TSHAstar::findPathTunnel(const Path::ReferencePath::Ptr & reference_path, std::vector<std::pair<double, double>> & tunnel_bounds)
{
    // 此处转换后全部都是以栅格为单位，和之前的坐标也保持一致
    const static double longitudial_sample_spacing = params_.sample.path_sample.LONGITUDIAL_SAMPLE_SPACING / res_;
    const static double lateral_sample_spacing     = params_.sample.path_sample.LATERAL_SAMPLE_SPACING / res_;
    const static double lateral_sample_range       = params_.sample.path_sample.LATERAL_SAMPLE_RANGE / res_;
    const static double dp_collision_distance      = params_.sample.path_dp.BOUND_CHECK_INTERVAL / res_;
    const static double dp_bound_check_interval    = params_.sample.path_dp.BOUND_CHECK_INTERVAL / res_;

    
    // 1. 沿参考系进行均匀纵向撒点，并记录撒点位置的状态信息
    auto time_t1 = std::chrono::steady_clock::now();
    std::vector<std::vector<SampleNode>> sample_nodes;
    // 1.1 对路径点进行均匀纵向撒点
    double accumulated_s = 0.0;
    double prev_s = accumulated_s;
    std::vector<double> s_list;
    while (accumulated_s < reference_path->GetLength())
    {
        accumulated_s += longitudial_sample_spacing;
        if (accumulated_s + longitudial_sample_spacing / 2.0 > reference_path->GetLength())     // 防止终点附近连续出现多个点
        {
            accumulated_s = reference_path->GetLength();
        }
        accumulated_s = std::min(accumulated_s, reference_path->GetLength());                   // 防止最后一个点超出参考线
        s_list.emplace_back(accumulated_s);
    }
    // 1.2 车辆固定是参考线的起点，因此撒点第一列特殊处理
    const Path::PathNode & start_path_node = reference_path->GetPathNode(0.0);
    SampleNode start_node;
    start_node.s = 0.0;
    start_node.l = 0.0;
    start_node.x = start_point_.x;
    start_node.y = start_point_.y;
    start_node.heading = start_yaw_;
    start_node.theta = start_path_node.theta;
    start_node.kappa = start_path_node.kappa;
    start_node.dis_to_obs = sample_map_.GetDistance(start_node.x, start_node.y);
    start_node.is_feasible = true;
    start_node.cost = 0.0;
    start_node.parent = nullptr;
    sample_nodes.emplace_back();
    sample_nodes.back().emplace_back(std::move(start_node));
    // 1.3 对路径点进行均匀切向撒点，并根据障碍物距离信息、曲线几何信息给出可通行判断
    // 外循环：获取纵向点
    for (size_t i = 1; i < s_list.size(); i++)
    {
        sample_nodes.emplace_back();
        const Path::PathNode ref_path_node = reference_path->GetPathNode(s_list[i]);

        // 内循环，获取切向点
        for (double accumulated_l = -lateral_sample_range;
            accumulated_l < lateral_sample_range;
            accumulated_l += lateral_sample_spacing)
        {
            SampleNode curr_node;
            Path::PointXY curr_xy = Path::Utils::SLtoXY(
                { ref_path_node.s, accumulated_l }, { ref_path_node.x, ref_path_node.y }, ref_path_node.theta);
            curr_node.s = ref_path_node.s;
            curr_node.l = accumulated_l;
            curr_node.x = curr_xy.x;
            curr_node.y = curr_xy.y;
            curr_node.theta = ref_path_node.theta;
            curr_node.kappa = ref_path_node.kappa;

            curr_node.dis_to_obs = sample_map_.IsInside(curr_node.x, curr_node.y) ?
                sample_map_.GetDistance(curr_node.x, curr_node.y) : -1.0;
            const double ref_r = 1 / ref_path_node.kappa;
            curr_node.is_feasible = !((curr_node.dis_to_obs < dp_collision_distance) ||         // 当前点离障碍物过近，则直接不可通过
                                      (ref_path_node.kappa < 0 && accumulated_l < ref_r) ||     // 或者撒点位置已经超过了路径的曲率中心（意味着点可能会倒退），也不可通过
                                      (ref_path_node.kappa > 0 && accumulated_l > ref_r));
            sample_nodes.back().emplace_back(curr_node);
        }
    }
    helper_.sample.path_sample.lon_points_size = sample_nodes.size();
    helper_.sample.path_sample.lat_points_size = sample_nodes.back().size();

    // 2. 使用动态规划在可通行的点之间寻找符合运动学约束的最小代价路径
    auto time_t2 = std::chrono::steady_clock::now();
    for (size_t lon_idx = 1; lon_idx < sample_nodes.size(); lon_idx++)
    {
        for (size_t lat_idx = 0; lat_idx < sample_nodes[lon_idx].size(); lat_idx++)
        {
            calCostInSampleNodes(sample_nodes, lon_idx, lat_idx);
        }
    }

    // 3. 从最后一列寻找代价最低的点作为终点
    SampleNode * end_node = nullptr;
    double min_cost = std::numeric_limits<double>::max();
    for (SampleNode & last_node : sample_nodes.back())
    {
        if (last_node.cost < min_cost)
        {
            min_cost = last_node.cost;
            end_node = &last_node;
        }
    }
    if (!end_node)
    {
        return false;
    }

    // 4. 回溯路径，从终点到起点，记录路径点确定的上下边界值
    auto time_t3 = std::chrono::steady_clock::now();
    tunnel_bounds.resize(sample_nodes.size());
    tunnel_bounds.clear();
    helper_.sample.path_dp.dp_path.resize(sample_nodes.size());
    helper_.sample.path_dp.dp_path.clear();
    helper_.sample.path_dp.lower_bound.resize(sample_nodes.size());
    helper_.sample.path_dp.lower_bound.clear();
    helper_.sample.path_dp.upper_bound.resize(sample_nodes.size());
    helper_.sample.path_dp.upper_bound.clear();
    while (end_node)
    {
        // 对应参考点
        const Path::PathNode ref_node = reference_path->GetPathNode(end_node->s);

        // 确定下边界
        double lower_bound = end_node->l;
        while (lower_bound > -lateral_sample_range)
        {
            lower_bound -= dp_bound_check_interval;
            Path::PointSL sl(end_node->s, lower_bound);
            Path::PointXY xy = Path::Utils::SLtoXY(sl, { ref_node.x, ref_node.y }, ref_node.theta);
            if (!sample_map_.IsInside(xy.x, xy.y) ||                            // 此处点已经不在地图内部
                sample_map_.GetDistance(xy.x, xy.y) < dp_collision_distance)    // 或者此处点发生了碰撞，则认为上次结果即为边界
            {
                lower_bound += dp_bound_check_interval;
                break;
            }
        }

        // 确定上边界
        double upper_bound = end_node->l;
        while (upper_bound < lateral_sample_range)
        {
            upper_bound += dp_bound_check_interval;
            Path::PointSL sl(end_node->s, upper_bound);
            Path::PointXY xy = Path::Utils::SLtoXY(sl, { ref_node.x, ref_node.y }, ref_node.theta);
            if (!sample_map_.IsInside(xy.x, xy.y) ||                            // 此处点已经不在地图内部
                sample_map_.GetDistance(xy.x, xy.y) < dp_collision_distance)    // 或者此处点发生了碰撞，则认为上次结果即为边界
            {
                upper_bound -= dp_bound_check_interval;
                break;
            }
        }
        
        // 保存结果数据
        tunnel_bounds.emplace_back(lower_bound, upper_bound);
        helper_.sample.path_dp.dp_path.emplace_back(end_node->x, end_node->y);
        Path::PointSL lower_sl(end_node->s, lower_bound);
        Path::PointXY lower_xy = Path::Utils::SLtoXY(lower_sl, { ref_node.x, ref_node.y }, ref_node.theta);
        helper_.sample.path_dp.lower_bound.emplace_back(lower_xy.x, lower_xy.y);
        Path::PointSL upper_sl(end_node->s, upper_bound);
        Path::PointXY upper_xy = Path::Utils::SLtoXY(upper_sl, { ref_node.x, ref_node.y }, ref_node.theta);
        helper_.sample.path_dp.upper_bound.emplace_back(upper_xy.x, upper_xy.y);
        end_node = end_node->parent;
    }
    std::reverse(tunnel_bounds.begin(), tunnel_bounds.end());
    std::reverse(helper_.sample.path_dp.dp_path.begin(), helper_.sample.path_dp.dp_path.end());
    std::reverse(helper_.sample.path_dp.lower_bound.begin(), helper_.sample.path_dp.lower_bound.end());
    std::reverse(helper_.sample.path_dp.upper_bound.begin(), helper_.sample.path_dp.upper_bound.end());
    for (size_t i = 0; i < helper_.sample.path_dp.dp_path.size(); ++i)
    {
        helper_.sample.path_dp.dp_path[i].x = helper_.sample.path_dp.dp_path[i].x * res_ + ori_x_;
        helper_.sample.path_dp.dp_path[i].y = helper_.sample.path_dp.dp_path[i].y * res_ + ori_y_;

        helper_.sample.path_dp.lower_bound[i].x = helper_.sample.path_dp.lower_bound[i].x * res_ + ori_x_;
        helper_.sample.path_dp.lower_bound[i].y = helper_.sample.path_dp.lower_bound[i].y * res_ + ori_y_;

        helper_.sample.path_dp.upper_bound[i].x = helper_.sample.path_dp.upper_bound[i].x * res_ + ori_x_;
        helper_.sample.path_dp.upper_bound[i].y = helper_.sample.path_dp.upper_bound[i].y * res_ + ori_y_;
    }
    

    auto time_t4 = std::chrono::steady_clock::now();
    helper_.sample.path_sample.cost_time   = (time_t2 - time_t1).count() / 1000000.0;
    helper_.sample.path_dp.dp_cost_time    = (time_t3 - time_t2).count() / 1000000.0;
    helper_.sample.path_dp.bound_cost_time = (time_t4 - time_t3).count() / 1000000.0;
    helper_.sample.path_dp.total_cost_time = (time_t4 - time_t2).count() / 1000000.0;
    return true;
}

bool TSHAstar::optimizePiecewiseJerkPath(const Path::ReferencePath::Ptr & reference_path, const std::vector<std::pair<double, double>> & bounds, std::vector<cv::Point2d> & optimized_path)
{
    // 设置权重等参数
    const static std::array<double, 4> lateral_weights   = { params_.sample.path_qp.WEIGHT_L,
                                                             params_.sample.path_qp.WEIGHT_DL,
                                                             params_.sample.path_qp.WEIGHT_DDL,
                                                             params_.sample.path_qp.WEIGHT_DDDL };
    const static double center_weight                    =   params_.sample.path_qp.WEIGHT_CENTER;
    const static std::array<double, 3> end_state_weights = { params_.sample.path_qp.WEIGHT_END_STATE_L,
                                                             params_.sample.path_qp.WEIGHT_END_STATE_DL,
                                                             params_.sample.path_qp.WEIGHT_END_STATE_DDL };

    const static double lateral_sample_range   = params_.sample.path_sample.LATERAL_SAMPLE_RANGE / res_;
    const static double dl_limit               = params_.sample.path_qp.DL_LIMIT;
    const static double vehicle_kappa_max      = params_.sample.path_qp.VEHICLE_KAPPA_MAX * res_;
    const static double center_deviation_thres = params_.sample.path_qp.CENTER_DEVIATION_THRESHOLD / res_;
    const static double center_bounds_thres    = params_.sample.path_qp.CENTER_BOUNDS_THRESHOLD / res_;
    const static double center_obs_coeff       = params_.sample.path_qp.CENTER_OBS_COEFFICIENT;

    Smoother::PiecewiseJerkSmoother smoother(lateral_weights, center_weight, end_state_weights,
                                             lateral_sample_range, dl_limit, vehicle_kappa_max,
                                             center_deviation_thres, center_bounds_thres, center_obs_coeff);

    // 传入求解数据进行求解
    optimized_path.clear();
    if (!smoother.Solve(reference_path, bounds, { 0.0, 0.0, 0.0 }, { 0.0, 0.0, 0.0 }, optimized_path))
    {
        return false;
    }

    // 转换坐标
    std::for_each(optimized_path.begin(), optimized_path.end(), [this](cv::Point2d & p)
        {
            p.x = p.x * res_ + ori_x_;
            p.y = p.y * res_ + ori_y_;
        });

    return true;
}
// ========================= TSHAstar =========================

// ⣿⣿⣿⠟⠛⠛⠻⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⡟⢋⣩⣉⢻⣿⣿⣿⣿
// ⣿⣿⣿⠀⣿⣶⣕⣈⠹⠿⠿⠿⠿⠟⠛⣛⢋⣰⠣⣿⣿⠀⣿⣿⣿⣿
// ⣿⣿⣿⡀⣿⣿⣿⣧⢻⣿⣶⣷⣿⣿⣿⣿⣿⣿⠿⠶⡝⠀⣿⣿⣿⣿
// ⣿⣿⣿⣷⠘⣿⣿⣿⢏⣿⣿⣋⣀⣈⣻⣿⣿⣷⣤⣤⣿⡐⢿⣿⣿⣿
// ⣿⣿⣿⣿⣆⢩⣝⣫⣾⣿⣿⣿⣿⡟⠿⠿⠦⠀⠸⠿⣻⣿⡄⢻⣿⣿
// ⣿⣿⣿⣿⣿⡄⢻⣿⣿⣿⣿⣿⣿⣿⣿⣶⣶⣾⣿⣿⣿⣿⠇⣼⣿⣿
// ⣿⣿⣿⣿⣿⣿⡄⢿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⡟⣰⣿⣿⣿
// ⣿⣿⣿⣿⣿⣿⠇⣼⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⢀⣿⣿⣿⣿
// ⣿⣿⣿⣿⣿⠏⢰⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⢸⣿⣿⣿⣿
// ⣿⣿⣿⣿⠟⣰⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⠀⣿⣿⣿⣿
// ⣿⣿⣿⠋⣴⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⡄⣿⣿⣿⣿
// ⣿⣿⠋⣼⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⡇⢸⣿⣿⣿
