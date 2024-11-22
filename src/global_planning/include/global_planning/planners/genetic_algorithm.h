#pragma once
#include <random>

#include "global_planning/planners/global_planner_interface.h"
#include "global_planning/tools/math.h"
#include "global_planning/tools/print_struct_and_enum.h"


/// @brief 基于遗传算法(GA, Genetic Algorithm)的全局路径规划器。
/// 随机点生成和后续使用都是离散点而非栅格点，然后放在栅格地图中进行障碍物判断。
/// 感觉这个思路废了。GA用来优化路径还可以，但是用GA规划路径和想象中的差别太大。因此GA目前效果很差很差，完全没法用，后续也可能不会再用。
class GA : public GlobalPlannerInterface
{
public:
    /// @brief 用于GA规划器使用的参数类型
    struct GAParams : public GlobalPlannerParams
    {
        ~GAParams() = default;

        // 地图相关参数，uint16_t类型原本为uint8_t类型，但cout输出uint8_t类型时按照ASCII码输出字符，而非整数。
        struct
        {
            uint16_t OBSTACLE_THRESHOLD = 50;   // 地图中栅格代价值大于等于(>=)该值的栅格，会被视为障碍物，搜索过程中将直接跳过该栅格

            REGISTER_STRUCT(REGISTER_MEMBER(OBSTACLE_THRESHOLD));
        } map;

        struct
        {
            size_t GENERATION_SIZE = 200;       // 种群迭代次数，此处指算法迭代次数
            size_t POPULATION_SIZE = 50;        // 种群数量，此处指每次迭代中将生成的待选路径的数量
            size_t CHROMOSOME_SIZE = 10;        // 染色体数目，此处指每条路径上的节点数目，不包括起点和终点
            double CROSSOVER_RATE = 0.7;        // 交叉概率
            double MUTATION_RATE = 0.01;        // 变异概率

            REGISTER_STRUCT(REGISTER_MEMBER(GENERATION_SIZE),
                            REGISTER_MEMBER(POPULATION_SIZE),
                            REGISTER_MEMBER(CHROMOSOME_SIZE),
                            REGISTER_MEMBER(CROSSOVER_RATE),
                            REGISTER_MEMBER(MUTATION_RATE));
        } optimization;

        REGISTER_STRUCT(REGISTER_MEMBER(map),
                        REGISTER_MEMBER(optimization));
    };

    /// @brief 用于GA规划器的辅助类，实现数据记录和结果打印
    class GAHelper : public GlobalPlannerHelper
    {
    public:
        using GlobalPlannerHelper::GlobalPlannerHelper;
        ~GAHelper() = default;

        struct
        {
            double cost_time = 0;           // 搜索总耗时，单位ms

            REGISTER_STRUCT(REGISTER_MEMBER(cost_time));
        } optimization;

    public:
        /// @brief 清空当前记录的所有结果信息，便于下次记录
        void resetResultInfo() override
        {
            optimization.cost_time = 0;
        }

    private:
        /// @brief 将规划器中设置的参数以字符串的形式输出
        /// @return 规划器的参数
        std::string paramsInfo() const override;
        /// @brief 将规划器所使用的地图信息、起点、终点以字符串的形式输出
        /// @return 地图信息、起点、终点信息
        std::string mapInfo() const override;
        /// @brief 将helper中保存的所有有关规划的结果以字符串的形式输出
        /// @return 规划的结果数值
        std::string resultInfo() const override;
    };

private:

public:
    GA() : helper_(this), rand_generator_(static_cast<uint_fast32_t>(std::time(nullptr))), dis_prob_(0.0, 1.0) { planner_name_ = "GA"; }
    GA(const GA & other) = delete;
    GA(GA && other) = delete;
    GA & operator=(const GA & other) = delete;
    GA & operator=(GA && other) = delete;
    ~GA() = default;

    /// @brief 对规划器相关变量进行初始化设置，进行参数拷贝设置
    /// @param params 传入的参数
    void initParams(const GlobalPlannerParams & params) override;
    /// @brief 对输入的map的进行二值化处理，然后设置成为规划器中需要使用的地图
    /// @param map 输入的原始地图
    /// @return 地图设置是否成功
    bool setMap(const cv::Mat & map) override;
    /// @brief 设置规划路径的起点。以真实地图坐标形式，而非行列形式。
    /// @param x 真实地图坐标系的x值
    /// @param y 真实地图坐标系的y值
    /// @return 该点是否能够成为起点。即该点在地图内部且不在障碍物上。
    bool setStartPoint(const double x, const double y) override;
    /// @brief 设置规划路径的起点。以真实地图坐标形式，而非行列形式。
    /// @param p 真实地图坐标系的点
    /// @return 该点是否能够成为起点。即该点在地图内部且不在障碍物上。
    bool setStartPoint(const cv::Point2d & p) override;
    /// @brief 设置规划路径的终点。以真实地图坐标形式，而非行列形式。
    /// @param x 真实地图坐标系的x值
    /// @param y 真实地图坐标系的y值
    /// @return 该点是否能够成为终点。即该点在地图内部且不在障碍物上。
    bool setEndPoint(const double x, const double y) override;
    /// @brief 设置规划路径的终点。以真实地图坐标形式，而非行列形式。
    /// @param p 真实地图坐标系的点
    /// @return 该点是否能够成为终点。即该点在地图内部且不在障碍物上。
    bool setEndPoint(const cv::Point2d & p) override;
    /// @brief 获得处理后的地图，即算法内部真正使用的，经过二值化后的地图
    /// @param map 地图将存入该变量
    /// @return 存入是否成功
    bool getProcessedMap(cv::Mat & map) const override;
    /// @brief 通过给定的地图、起点、终点规划出一条从起点到终点的最终路径。
    /// @param path 规划出的路径。该路径是原始地图坐标系下原始路径点。
    /// @param auxiliary_info 辅助信息。该信息是原始地图坐标系下路径规划过程中的各种关键路径点信息。此处包括RRT扩展到的其他节点。
    /// @return 是否规划成功
    bool getPath(std::vector<cv::Point2d> & path, std::vector<std::vector<cv::Point2d>> & auxiliary_info) override;
    /// @brief 打印所有的信息，包括规划器参数信息、规划地图信息、规划结果信息，并可以将结果保存到指定路径中。调用内部辅助helper实现
    /// @param save 是否保存到本地
    /// @param save_dir_path 保存的路径
    void showAllInfo(const bool save = false, const std::string & save_dir_path = "") const override { helper_.showAllInfo(save, save_dir_path); }

private:
    GAParams params_;                   // 规划器参数
    GAHelper helper_;                   // 规划器辅助

    cv::Mat map_;                       // 代价地图
    cv::Point2d start_point_;           // 起点，真实地图上的起点落在栅格地图中的离散坐标
    cv::Point2d end_point_;             // 终点，真实地图上的终点落在栅格地图中的离散坐标

    std::mt19937 rand_generator_;                       // 随机数生成器
    std::uniform_real_distribution<double> dis_row_;    // 随机数分布器，用于生成随机的行坐标
    std::uniform_real_distribution<double> dis_col_;    // 随机数分布器，用于生成随机的列坐标
    std::uniform_real_distribution<double> dis_prob_;   // 随机数分布器，用于生成0~1的随机数，用于交叉概率和变异概率等概率的选择

    std::vector<std::vector<cv::Point2d>> population_;  // 种群，每条路径是一个个体，路径上的节点数目时染色体数目。
    std::vector<uint8_t> is_collision_;                 // 碰撞信息，记录每条路径上是否发生碰撞。
    std::vector<double> fitness_;                       // 适应度，每条路径的适应度值，越小越好。
    
    /// @brief 生成随机点，落在栅格地图中，且不在障碍物上
    /// @return 生成的随机点
    cv::Point2d get_random_point();
    /// @brief 地图上检测pt1和pt2连线上是否存在障碍物节点。
    /// @param pt1 第一个点
    /// @param pt2 第二个点
    /// @return 是否发生碰撞。true —— 碰撞； false —— 不碰撞
    bool check_collision(const cv::Point2d & pt1, const cv::Point2d & pt2) const;
    /// @brief 初始化种群。所有路径都随机生成，且路径上节点数目时染色体数目。中间路径无碰撞，中间路径与终点不保证是否碰撞。
    void init_population();
    /// @brief 计算适应度。适应度值越小越好。
    void calc_fitness();
    /// @brief 选择。按照父代适应度比例，生成子代。
    /// @return 根据适应度选择生成的子代。用原有父代的索引表示
    std::vector<size_t> selection();
    /// @brief 交叉。根据select生成的子代，进行交叉。
    /// @return 交叉后的子代。
    std::vector<std::vector<cv::Point2d>> crossover(std::vector<size_t> && selected_indexes);
    /// @brief 变异。对交叉后的子代进行变异。结果直接更新为新的种群。
    void mutation(std::vector<std::vector<cv::Point2d>> && offspring);
    /// @brief 计算碰撞信息。根据路径上的节点，计算是否发生碰撞。
    void calc_collision();
    /// @brief 得到最终路径。在种群迭代完毕后，选择适应度最好的路径作为最终路径。
    /// @param path 最终路径。
    /// @return 如果最终路径会碰撞，则返回false；否则返回true。
    bool final_path(std::vector<cv::Point2d> & path) const;
};