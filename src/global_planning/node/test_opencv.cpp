#include <ros/ros.h>
#include <ros/package.h>

#include "global_planning/planners/DVAstar.h"
#include "global_planning/planners/astar.h"
#include "global_planning/planners/rrt.h"
#include "global_planning/planners/rrtstar.h"
#include "global_planning/planners/genetic_algorithm.h"
#include "global_planning/tools/map_generator.h"

int main(int argc, char * argv[])
{
    ros::init(argc, argv, "test_opencv");
    ros::NodeHandle nh("~");

    GlobalPlannerInterface * planner;
    std::string planner_name = nh.param<std::string>("planner_name", "DVAstar");
    if (planner_name == "DVAstar")
    {
        // DVAstar规划器
        DVAstar::DVAstarParams DVAstar_params;
        DVAstar_params.map_params.EXPANDED_K = 1.3;
        DVAstar_params.map_params.EXPANDED_MIN_THRESHOLD = 0;
        DVAstar_params.map_params.EXPANDED_MAX_THRESHOLD = 100;
        DVAstar_params.map_params.COST_THRESHOLD = 10;
        DVAstar_params.map_params.OBSTACLE_THRESHOLD = 100;
        DVAstar_params.cost_function_params.NEIGHBOR_TYPE = DVAstar::NeighborType::FiveConnected;
        DVAstar_params.cost_function_params.HEURISTICS_TYPE = DVAstar::HeuristicsType::Euclidean;
        DVAstar_params.cost_function_params.TRAV_COST_K = 2.0;
        DVAstar_params.cost_function_params.TURN_COST_STRAIGHT = 1.0;
        DVAstar_params.cost_function_params.TURN_COST_SLANT = 1.4;
        DVAstar_params.cost_function_params.TURN_COST_VERTICAL = 2.0;
        DVAstar_params.cost_function_params.TURN_COST_REVERSE_SLANT = 3.0;
        DVAstar_params.path_simplification_params.PATH_SIMPLIFICATION_TYPE = DVAstar::PathSimplificationType::DPPlus;
        DVAstar_params.path_simplification_params.DISTANCE_THRESHOLD = 18;
        DVAstar_params.path_simplification_params.ANGLE_THRESHOLD = 10 / 180 * M_PI;
        DVAstar_params.path_simplification_params.OBSTACLE_THRESHOLD = 70;
        DVAstar_params.path_simplification_params.LINE_WIDTH = 15;
        DVAstar_params.path_simplification_params.MAX_INTAVAL = 120.0;
        DVAstar_params.path_smooth_params.PATH_SMOOTH_TYPE = DVAstar::PathSmoothType::BSpline;
        DVAstar_params.path_smooth_params.T_STEP = 0.0005;
        DVAstar_params.downsampling_params.INTERVAL = 2;
        planner = new DVAstar;
        planner->initParams(DVAstar_params);
    }
    else if (planner_name == "Astar")
    {
        // Astar规划器
        Astar::AstarParams astar_params;
        astar_params.map_params.OBSTACLE_THRESHOLD = 50;
        astar_params.cost_function_params.HEURISTICS_TYPE = Astar::HeuristicsType::Euclidean;
        planner = new Astar;
        planner->initParams(astar_params);
    }
    else if (planner_name == "RRT")
    {
        // RRT规划器
        RRT::RRTParams rrt_params;
        rrt_params.map_params.OBSTACLE_THRESHOLD = 50;
        rrt_params.sample_params.ITERATOR_TIMES = 10000;
        rrt_params.sample_params.GOAL_SAMPLE_RATE = 0.05;
        rrt_params.sample_params.GOAL_DIS_TOLERANCE = 20;
        rrt_params.sample_params.STEP_SIZE = 40;
        planner = new RRT;
        planner->initParams(rrt_params);
    }
    else if (planner_name == "RRTstar")
    {
        // RRTstar规划器
        RRTstar::RRTstarParams rrtstar_params;
        rrtstar_params.map_params.OBSTACLE_THRESHOLD = 50;
        rrtstar_params.sample_params.ITERATOR_TIMES = 10000;
        rrtstar_params.sample_params.GOAL_SAMPLE_RATE = 0.05;
        rrtstar_params.sample_params.GOAL_DIS_TOLERANCE = 20;
        rrtstar_params.sample_params.STEP_SIZE = 40;
        rrtstar_params.sample_params.NEAR_DIS = 80;
        planner = new RRTstar;
        planner->initParams(rrtstar_params);
    }
    else if (planner_name == "GA")
    {
        // GA规划器
        GA::GAParams ga_params;
        ga_params.map_params.OBSTACLE_THRESHOLD = 50;
        ga_params.optimization_params.GENERATION_SIZE = 1000;
        ga_params.optimization_params.POPULATION_SIZE = 200;
        ga_params.optimization_params.CHROMOSOME_SIZE = 2;
        ga_params.optimization_params.CROSSOVER_RATE = 0.7;
        ga_params.optimization_params.MUTATION_RATE = 0.01;
        planner = new GA;
        planner->initParams(ga_params);
    }

    MapGenerator generator;

    // 0.颜色测试
    // generator.color_test();

    // 1.随机生成地图与Astar测试
    // generator.generate_random_map(50, 50, 0.2f);
    // generator.set_planner(&astar_planner);
    // generator.show_map("random map and astar test");

    // 2. 手动生成地图测试
    // 2.1 生成地图
    // generator.generate_blank_map(50, 50);
    // generator.add_obstacles(cv::Point2i(5, 2));                                         // 单个点
    // generator.add_obstacles({cv::Point2i(1, 4), cv::Point2i(4, 1), cv::Point2i(3, 3)}); // 多个点
    // generator.add_obstacles(cv::Point2i(0, 20), cv::Point2i(49, 20));                   // 直线
    // generator.add_obstacles(cv::Point2i(0, 20), cv::Point2i(3, 49));                    // 斜直线
    // generator.add_obstacles(cv::Rect(cv::Point2i(30, 30), cv::Point2i(40, 40)));        // 矩形
    // generator.add_obstacles(cv::Point2i(44, 44), 5);                                    // 圆形
    // 2.2 测试地图
    // generator.set_planner(&astar_planner);
    // 2.3 测试保存地图
    // generator.save_map(ros::package::getPath("global_planning") + "/map/map1.png");
    // 2.4 测试读取地图
    // generator.load_map(ros::package::getPath("global_planning") + "/map/map1.png");
    // generator.show_map("blank map and add obstacles test");

    // 3.生成所需要的地图
    // generator.generate_blank_map(100, 100);
    // generator.add_obstacles(cv::Point2i(0,  0),  cv::Point(99, 0));
    // generator.add_obstacles(cv::Point2i(99, 0),  cv::Point(99, 99));
    // generator.add_obstacles(cv::Point2i(99, 99), cv::Point(0,  99));
    // generator.add_obstacles(cv::Point2i(0,  99), cv::Point(0,  0));

    // generator.add_obstacles(cv::Point2i(0, 21), cv::Point(42, 21));
    // generator.add_obstacles(cv::Point2i(60, 0), cv::Point(60, 60));
    // generator.add_obstacles(cv::Point2i(20, 40), cv::Point(60, 40));
    // generator.add_obstacles(cv::Point2i(45, 10), cv::Point(45, 30));
    // generator.add_obstacles(cv::Point2i(16, 30), cv::Point(16, 50));
    // generator.add_obstacles(cv::Rect(cv::Point2i(0, 60), cv::Point(45, 82)));
    // generator.add_obstacles(cv::Point2i(60, 75), cv::Point(60, 99));
    // generator.add_obstacles(cv::Point2i(60, 75), cv::Point(85, 75));
    // generator.add_obstacles(cv::Point2i(75, 88), cv::Point(99, 88));
    // generator.add_obstacles(cv::Point2i(75, 75), cv::Point(75, 20));
    // generator.add_obstacles(cv::Point2i(70, 10), cv::Point(90, 10));
    // generator.add_obstacles(cv::Point2i(90, 60), cv::Point(90, 10));

    // generator.save_map(ros::package::getPath("global_planning") + "/map/map1.png");
    // generator.set_planner(&astar_planner);
    // generator.show_map("generate map");

    // 4.测试最终功能
    generator.load_map(ros::package::getPath("global_planning") + "/map/map1.png");
    generator.set_result_path(ros::package::getPath("global_planning") + "/result/test_result/");
    generator.set_planner(planner);
    generator.show_map("test", 10);

    delete planner;
    return 0;
}
