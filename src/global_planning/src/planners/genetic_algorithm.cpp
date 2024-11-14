#include "global_planning/planners/genetic_algorithm.h"


// ========================= GA::GAHelper =========================

std::string GA::GAHelper::paramsInfo() const
{
    const GA * ga_planner = dynamic_cast<const GA *>(planner_);
    const GAParams & ga_params = ga_planner->params_;

    std::stringstream params_info;
    params_info << "[Params Info]:\n";
    PRINT_STRUCT(params_info, ga_params);
    return params_info.str();
}

std::string GA::GAHelper::mapInfo() const
{
    const GA * ga_planner = dynamic_cast<const GA *>(planner_);

    std::stringstream map_info;
    map_info << "[Map Info]:\n"
             << "  rows: " << ga_planner->rows_ << std::endl
             << "  cols: " << ga_planner->cols_ << std::endl
             << "  resolution: " << ga_planner->res_ << std::endl
             << "  start point: " << ga_planner->start_point_ << std::endl
             << "  end point:   " << ga_planner->end_point_ << std::endl;
    return map_info.str();
}

std::string GA::GAHelper::resultInfo() const
{
    std::stringstream result_info;
    result_info << "[Result Info]:\n";
    PRINT_STRUCT(result_info, optimization_result);
    return result_info.str();
}
// ========================= GA::GAHelper =========================


// ========================= GA =========================

void GA::initParams(const GlobalPlannerParams & params)
{
    const GAParams & p = dynamic_cast<const GAParams &>(params);
    params_ = p;
}

bool GA::setMap(const cv::Mat & map)
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
            if (cost < params_.map_params.OBSTACLE_THRESHOLD)
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

    // 设置随机数
    dis_row_.param(std::uniform_real_distribution<double>::param_type(0, rows_));
    dis_col_.param(std::uniform_real_distribution<double>::param_type(0, cols_));

    // printf("地图设置成功，大小 %d x %d\n", rows_, cols_);
    init_map_ = true;
    return true;
}

bool GA::setStartPoint(const double x, const double y)
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

bool GA::setStartPoint(const cv::Point2d & p)
{
    return setStartPoint(p.x, p.y);
}

bool GA::setEndPoint(const double x, const double y)
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

bool GA::setEndPoint(const cv::Point2d & p)
{
    return setEndPoint(p.x, p.y);
}

bool GA::getProcessedMap(cv::Mat & map) const
{
    if (!init_map_)
    {
        std::cout << "当前无有效地图！\n";
        return false;
    }

    map = map_.clone();
    return true;
}

bool GA::getPath(std::vector<cv::Point2d> & path, std::vector<std::vector<cv::Point2d>> & auxiliary_info)
{
    // 0.初始化helper
    helper_.resetResultInfo();
    auto start_time = std::chrono::steady_clock::now();

    // 1. 初始化种群
    init_population();

    // 2. 计算适应度
    calc_fitness();

    // 3. 循环迭代
    for (size_t iter = 0; iter < params_.optimization_params.GENERATION_SIZE; iter++)
    {
        // 3.1 选择
        std::vector<size_t> selected_idx = selection();

        // 3.2 交叉
        std::vector<std::vector<cv::Point2d>> offspring = crossover(std::move(selected_idx));

        // 3.3 变异
        mutation(std::move(offspring));

        // 3.4 计算碰撞
        calc_collision();

        // 3.5 计算适应度
        calc_fitness();
    }

    // 4. 选择最后子代中最优个体为路径
    size_t best_idx = 0;
    for (size_t i = 1; i < population_.size(); i++)
    {
        if (fitness_[i] > fitness_[best_idx])
        {
            best_idx = i;
        }
    }
    path.clear();
    for (const cv::Point2d & p : population_[best_idx])
    {
        path.push_back(cv::Point2d(p.x * res_ + ori_x_, p.y * res_ + ori_y_));
    }

    // 5. 计算辅助信息
    auto end_time = std::chrono::steady_clock::now();
    helper_.optimization_result.cost_time = (end_time - start_time).count() / 1000000.0;  // 算法耗时 ms

    return is_collision_[best_idx] == false;
}


cv::Point2d GA::get_random_point()
{
    cv::Point2d p;
    while (true)
    {
        p.x = dis_col_(rand_generator_);
        p.y = dis_row_(rand_generator_);
        cv::Point2i p_grid(static_cast<int>(p.x), static_cast<int>(p.y));
        if (map_.at<uchar>(p_grid) < params_.map_params.OBSTACLE_THRESHOLD)
        {
            return p;
        }
    }
}

bool GA::check_collision(const cv::Point2d & pt1, const cv::Point2d & pt2) const
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

void GA::init_population()
{
    population_.resize(params_.optimization_params.POPULATION_SIZE);
    is_collision_.resize(population_.size(), false);

    for (size_t i = 0; i < population_.size(); i++)
    {
        std::vector<cv::Point2d> individual(params_.optimization_params.CHROMOSOME_SIZE + 2);   // 单个个体，即一条路径，需要包含起点终点

        // 终点与中间路径不进行碰撞检测
        individual[0] = start_point_;
        individual[params_.optimization_params.CHROMOSOME_SIZE + 1] = end_point_;
        for (size_t i = 1; i <= params_.optimization_params.CHROMOSOME_SIZE; i++)
        {
            while (true)
            {
                cv::Point2d p = get_random_point();
                if (check_collision(individual[i - 1], p) == false)
                {
                    individual[i] = p;
                    break;
                }
            }
        }

        is_collision_[i] = check_collision(individual[params_.optimization_params.CHROMOSOME_SIZE + 1], individual[params_.optimization_params.CHROMOSOME_SIZE]);
        population_[i] = std::move(individual);
    }
}

void GA::calc_fitness()
{
    fitness_.resize(population_.size(), 0.0);
    for (size_t i = 0; i < population_.size(); i++)
    {
        if (is_collision_[i] == true)   // 若碰撞，则适应度为0
        {
            fitness_[i] = 0.0;
        }
        else
        {
            const std::vector<cv::Point2d> & individual = population_[i];

            // 1. 距离适应度
            double length = 0.0;
            for (size_t j = 1; j < individual.size(); j++)
            {
                length += std::hypot(individual[j].x - individual[j - 1].x, individual[j].y - individual[j - 1].y);
            }
            fitness_[i] += 1.0 / length;
        
            // 2. 角度适应度
            // for (size_t j = 1; j < individual.size() - 1; j++)
            // {
            //     double dx1 = individual[j].x - individual[j - 1].x;
            //     double dy1 = individual[j].y - individual[j - 1].y;
            //     double dx2 = individual[j + 1].x - individual[j].x;
            //     double dy2 = individual[j + 1].y - individual[j].y;
            //     double angle = std::atan2(dy1 * dx2 - dx1 * dy2, dx1 * dx2 + dy1 * dy2);
            //     fitness_[i] += 0.1 / (1.0 + std::abs(angle));
            // }
        }
    }
    
}

std::vector<size_t> GA::selection()
{
    std::vector<size_t> selected_idx(population_.size());

    // 保留最优个体，剩余使用轮盘赌选择
    const double sum_fitness = std::accumulate(fitness_.begin(), fitness_.end(), 0.0);
    std::vector<double> prob(population_.size());
    double max_prob = 0.0;
    double max_idx = 0;
    for (size_t i = 0; i < population_.size(); i++)
    {
        prob[i] = fitness_[i] / sum_fitness;
        if (prob[i] > max_prob)
        {
            max_prob = prob[i];
            max_idx = i;
        }
    }
    selected_idx[0] = max_idx;  // 保留最优个体

    std::vector<double> cum_prob(population_.size());
    cum_prob[0] = prob[0];
    for (size_t i = 1; i < population_.size(); i++)
    {
        cum_prob[i] = cum_prob[i - 1] + prob[i];
    }

    for (size_t i = 1; i < population_.size(); i++) // 剩余个体进行轮盘赌选择
    {
        const double rand_num = dis_prob_(rand_generator_);
        for (size_t j = 0; j < population_.size(); j++)
        {
            if (rand_num <= cum_prob[j])
            {
                selected_idx[i] = j;
                break;
            }
        }
    }

    return selected_idx;
}

std::vector<std::vector<cv::Point2d>> GA::crossover(std::vector<size_t> && selected_indexes)
{
    std::vector<std::vector<cv::Point2d>> offspring(population_.size());

    for (size_t i = 0; i < selected_indexes.size(); i++)
    {
        const size_t parent1_idx = selected_indexes[i];
        const size_t parent2_idx = (selected_indexes[i] + 1) % population_.size();

        const std::vector<cv::Point2d> & parent1 = population_[parent1_idx];
        const std::vector<cv::Point2d> & parent2 = population_[parent2_idx];

        std::vector<cv::Point2d> & offspring_i = offspring[i];
        offspring_i.resize(params_.optimization_params.CHROMOSOME_SIZE + 2);   // 单个个体，即一条路径，需要包含起点终点

        // 判断是否交叉
        if (dis_prob_(rand_generator_) < params_.optimization_params.CROSSOVER_RATE)
        {
            // 1. 随机选择交叉点
            size_t cross_idx = dis_prob_(rand_generator_) * (params_.optimization_params.CHROMOSOME_SIZE + 2);

            // 2. 交叉
            for (size_t j = 0; j < offspring_i.size(); j++)
            {
                if (j < cross_idx)
                {
                    offspring_i[j] = parent1[j];
                }
                else // if (j > cross_idx)
                {
                    offspring_i[j] = parent2[j];
                }
            }
        }
        else
        {
            offspring_i = parent1;
        }
    }

    return offspring;
}

void GA::mutation(std::vector<std::vector<cv::Point2d>> && offspring)
{
    for (size_t i = 0; i < offspring.size(); i++)
    {
        std::vector<cv::Point2d> & individual = offspring[i];

        // 1. 随机选择变异点
        std::vector<size_t> mut_idx;
        for (size_t j = 1; j < individual.size() - 1; j++)
        {
            if (dis_prob_(rand_generator_) < params_.optimization_params.MUTATION_RATE)
            {
                mut_idx.push_back(j);
            }
        }
        
        // 2. 变异    
        for (size_t j = 0; j < mut_idx.size(); j++)
        {
            while (true)
            {
                cv::Point2d p = get_random_point();
                if (check_collision(individual[mut_idx[j]], p) == false)
                {
                    individual[mut_idx[j]] = p;
                    break;
                }
            }
        }
    }

    population_ = std::move(offspring);
}

void GA::calc_collision()
{
    is_collision_.resize(population_.size(), false);
    for (size_t i = 0; i < population_.size(); i++)
    {
        const std::vector<cv::Point2d> & individual = population_[i];
        for (size_t j = 0; j < individual.size() - 1; j++)
        {
            if (check_collision(individual[j], individual[j + 1]))
            {
                is_collision_[i] = true;
                break;
            }
        }
    }
}
// ========================= GA =========================
