#include "global_planning/planners/QHAstar.h"


// ========================= QHAstar::QHAstarHelper =========================

std::string QHAstar::QHAstarHelper::paramsInfo() const
{
    const QHAstar * QHAstar_planner = dynamic_cast<const QHAstar *>(planner_);
    const QHAstarParams & QHAstar_params = QHAstar_planner->params_;

    std::stringstream params_info;
    params_info << "[Params Info]:\n";
    PRINT_STRUCT(params_info, QHAstar_params);
    return params_info.str();
}

std::string QHAstar::QHAstarHelper::mapInfo() const
{
    const QHAstar * QHAstar_planner = dynamic_cast<const QHAstar *>(planner_);

    std::stringstream map_info;
    map_info << "[Map Info]:\n"
        << "  rows: " << QHAstar_planner->rows_ << std::endl
        << "  cols: " << QHAstar_planner->cols_ << std::endl
        << "  resolution: " << QHAstar_planner->res_ << std::endl
        << "  start point: " << QHAstar_planner->start_node_->point << std::endl
        << "  end point:   " << QHAstar_planner->end_node_->point << std::endl;
    return map_info.str();
}

std::string QHAstar::QHAstarHelper::resultInfo() const
{
    std::stringstream result_info;
    result_info << "[Result Info]:\n";
    PRINT_STRUCT(result_info, path_planning);
    PRINT_STRUCT(result_info, path_smoothing);
    return result_info.str();
}
// ========================= QHAstar::QHAstarHelper =========================


// ========================= QHstar =========================

void QHAstar::initParams(const GlobalPlannerParams & params)
{
    const QHAstarParams & p = dynamic_cast<const QHAstarParams &>(params);
    params_ = p;
}

bool QHAstar::setMap(const cv::Mat & map)
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
            node.type = Node::NodeType::UNKNOWN;
            row[j] = std::move(node);
        }
        map_[i] = std::move(row);
    }
    cv::Mat bin_map;
    cv::threshold(map, bin_map, 99, 255, cv::ThresholdTypes::THRESH_BINARY_INV);
    cv::distanceTransform(bin_map, distance_map_, cv::DIST_L2, cv::DIST_MASK_PRECISE, CV_32FC1);
    max_dis_ = 0;
    min_dis_ = std::numeric_limits<double>::max();
    for (int i = 0; i < distance_map_.rows; i++)
    {
        for (int j = 0; j < distance_map_.cols; j++)
        {
            max_dis_ = std::max<double>(max_dis_, distance_map_.at<float>(i, j));
            min_dis_ = std::min<double>(min_dis_, distance_map_.at<float>(i, j));
        }
    }

    init_map_ = true;
    return true;
}

bool QHAstar::setStartPoint(const double x, const double y)
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

    if (map_[y_grid][x_grid].cost >= 100)
    {
        std::cout << "起点必须设置在非障碍物处！" << '\n';
        init_start_point_ = false;
        return false;
    }

    start_node_ = &map_[y_grid][x_grid];
    init_start_point_ = true;
    return true;
}

bool QHAstar::setStartPoint(const cv::Point2d & p)
{
    return setStartPoint(p.x, p.y);
}

bool QHAstar::setEndPoint(const double x, const double y)
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

    if (map_[y_grid][x_grid].cost >= 100)
    {
        std::cout << "终点必须设置在非障碍物处！\n";
        init_end_point_ = false;
        return false;
    }

    end_node_ = &map_[y_grid][x_grid];
    init_end_point_ = true;
    return true;
}

bool QHAstar::setEndPoint(const cv::Point2d & p)
{
    return setEndPoint(p.x, p.y);
}

bool QHAstar::getProcessedMap(cv::Mat & map) const
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

bool QHAstar::getPath(std::vector<cv::Point2d> & path, std::vector<std::vector<cv::Point2d>> & auxiliary_info)
{
    if (!(init_map_ && init_start_point_ && init_end_point_ && init_map_info_))
    {
        std::cout << "请先设置地图、起点和终点！\n";
        return false;
    }

    // 0.初始化helper
    helper_.resetResultInfo();

    // 1.生成路径
    std::vector<cv::Point2d> raw_path;
    PathPlanning(raw_path);

    // 2.平滑路径
    PathSmoothing(raw_path, path);

    return true;
}


double QHAstar::getH(const cv::Point2i & p) const
{
    double h = std::sqrt(std::pow(p.x - end_node_->point.x, 2) + std::pow(p.y - end_node_->point.y, 2));
    return h;
}

bool QHAstar::PathPlanning(std::vector<cv::Point2d> & path)
{
    // 初始节点
    auto start_time = std::chrono::steady_clock::now();
    start_node_->g = 0;
    start_node_->h = getH(start_node_->point);
    start_node_->f = start_node_->g + start_node_->h;
    start_node_->parent_node = nullptr;
    std::priority_queue<Node *, std::vector<Node *>, Node::NodePointerCmp> queue;
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
                
                if (new_node->cost < 100 &&                     // 保证该节点是非障碍物节点，才能联通。代价地图[0, 100] 0可通过 100不可通过 
                    new_node->type != Node::NodeType::CLOSED)   // 不对已加入CLOSED的数据再次判断
                {
                    // QHAstar: au 增加代价系数。根据栅格代价归一化后得到的系数
                    const double u_n = new_node->cost / 100.0;
                    const double au = 1.0 / (1.0 - u_n + 1e-6);
                    // QHAstar: ad 增加障碍物距离系数。根据与障碍物距离（归一化后）得到的系数。
                    const double d_n = (distance_map_.at<float>(new_point) - min_dis_) / (max_dis_ - min_dis_);
                    const double ad = 1.0 / d_n;
                    // QHAstar: G是 dis * au * ad
                    const double dis = std::hypot(new_point.x - this_point.x, new_point.y - this_point.y) * au * ad;

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
        path.push_back(cv::Point2d((path_node->point.x + 0.5) * res_ + ori_x_,
            (path_node->point.y + 0.5) * res_ + ori_y_));
        path_node = path_node->parent_node;
    }
    std::reverse(path.begin(), path.end());

    // 保存结果信息
    auto end_time = std::chrono::steady_clock::now();
    helper_.path_planning.cost_time = (end_time - start_time).count() / 1000000.0;  // 算法耗时 ms

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

bool QHAstar::PathSmoothing(const std::vector<cv::Point2d> & raw_path, std::vector<cv::Point2d> & path)
{
    auto start_time = std::chrono::steady_clock::now();
    
    // 使用离散点对参考线通过数值优化进行平滑。
    const std::array<double, 3> weights = { params_.smoothing.WEIGTH_SMOOTH,
                                            params_.smoothing.WEIGTH_LENGTH,
                                            params_.smoothing.WEIGTH_DEVIATION };
    Smoother::DiscretePointSmoother smoother(weights, res_ / 2.0); // QHAstar： buffer_为0.2，是栅格分辨率的一半
    // QHAstar: 使用分段滚动平滑
    // 40是每次实际优化40个点；
    // 39是40 - 1得到的，因为这函数实际会优化end_idx位置的点；
    // 11是论文中前向固定的10个点+论文中所谓的起点固定1个点
    for (int start_idx = 0; start_idx < raw_path.size(); start_idx += 40)
    {
        int end_idx = std::min(start_idx + 39, static_cast<int>(raw_path.size()) - 1);
        if (!smoother.Solve(raw_path, path, start_idx, end_idx, 11))
        {
            return false;
        }
    }

    auto end_time = std::chrono::steady_clock::now();
    helper_.path_smoothing.cost_time = (end_time - start_time).count() / 1000000.0;  // 算法耗时 ms

    return true;
}

// ========================= QHstar =========================
