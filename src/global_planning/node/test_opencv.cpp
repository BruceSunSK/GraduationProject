#include <ros/ros.h>
#include <ros/package.h>

#include "global_planning/planners/TSHAstar.h"
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
    std::string planner_name = nh.param<std::string>("planner_name", "TSHAstar");
    if (planner_name == "TSHAstar")
    {
        // TSHAstar规划器
        TSHAstar::TSHAstarParams TSHAstar_params;
        TSHAstar_params.map.KERNEL_SIZE = 15;
        TSHAstar_params.map.EXPANDED_K = 1.3;
        TSHAstar_params.map.EXPANDED_MIN_THRESHOLD = 0;
        TSHAstar_params.map.EXPANDED_MAX_THRESHOLD = 100;
        TSHAstar_params.map.COST_THRESHOLD = 10;
        TSHAstar_params.map.OBSTACLE_THRESHOLD = 100;
        TSHAstar_params.search.path_search.NEIGHBOR_TYPE = TSHAstar::NeighborType::FiveConnected;
        TSHAstar_params.search.path_search.HEURISTICS_TYPE = TSHAstar::HeuristicsType::Euclidean;
        TSHAstar_params.search.path_search.TRAV_COST_K = 2.0;
        TSHAstar_params.search.path_search.TURN_COST_STRAIGHT = 1.0;
        TSHAstar_params.search.path_search.TURN_COST_SLANT = 1.4;
        TSHAstar_params.search.path_search.TURN_COST_VERTICAL = 2.0;
        TSHAstar_params.search.path_search.TURN_COST_REVERSE_SLANT = 3.0;
        TSHAstar_params.search.path_simplification.PATH_SIMPLIFICATION_TYPE = TSHAstar::PathSimplificationType::DPPlus;
        TSHAstar_params.search.path_simplification.DISTANCE_THRESHOLD = 18;
        TSHAstar_params.search.path_simplification.ANGLE_THRESHOLD = 10 / 180 * M_PI;
        TSHAstar_params.search.path_simplification.OBSTACLE_THRESHOLD = 70;
        TSHAstar_params.search.path_simplification.LINE_WIDTH = 15;
        TSHAstar_params.search.path_simplification.MAX_INTAVAL = 120.0;
        TSHAstar_params.search.path_smooth.PATH_SMOOTH_TYPE = TSHAstar::PathSmoothType::BSpline;
        TSHAstar_params.search.path_smooth.T_STEP = 0.0005;
        TSHAstar_params.search.path_optimization.S_INTERVAL = 4.0;
        TSHAstar_params.search.path_optimization.DUBINS_RADIUS = 2.5;
        TSHAstar_params.search.path_optimization.DUBINS_INTERVAL = 1.5;
        TSHAstar_params.search.path_optimization.DUBINS_LENGTH = 8.0;
        TSHAstar_params.search.path_optimization.REF_WEIGTH_SMOOTH = 100.0;
        TSHAstar_params.search.path_optimization.REF_WEIGTH_LENGTH = 1.0;
        TSHAstar_params.search.path_optimization.REF_WEIGTH_DEVIATION = 10.0;
        TSHAstar_params.search.path_optimization.REF_BUFFER_DISTANCE = 3.0;
        TSHAstar_params.sample.path_sample.LONGITUDIAL_SAMPLE_SPACING = 10.0;
        TSHAstar_params.sample.path_sample.LATERAL_SAMPLE_SPACING = 10.0;
        TSHAstar_params.sample.path_sample.LATERAL_SAMPLE_RANGE = 100.0;
        TSHAstar_params.sample.path_dp.COLLISION_DISTANCE = 20;
        TSHAstar_params.sample.path_dp.WARNING_DISTANCE = 80;
        TSHAstar_params.sample.path_dp.BOUND_CHECK_INTERVAL = 5;
        TSHAstar_params.sample.path_dp.WEIGHT_OFFSET = 50.0;
        TSHAstar_params.sample.path_dp.WEIGHT_OBSTACLE = 100.0;
        TSHAstar_params.sample.path_dp.WEIGHT_ANGLE_CHANGE = 1000.0;
        TSHAstar_params.sample.path_dp.WEIGHT_ANGLE_DIFF = 1.0;
        TSHAstar_params.sample.path_qp.WEIGHT_L = 1.0;
        TSHAstar_params.sample.path_qp.WEIGHT_DL = 100.0;
        TSHAstar_params.sample.path_qp.WEIGHT_DDL = 1000.0;
        TSHAstar_params.sample.path_qp.WEIGHT_DDDL = 7000.0;
        TSHAstar_params.sample.path_qp.WEIGHT_CENTER = 0.6;
        TSHAstar_params.sample.path_qp.WEIGHT_END_STATE_L = 10.0;
        TSHAstar_params.sample.path_qp.WEIGHT_END_STATE_DL = 50.0;
        TSHAstar_params.sample.path_qp.WEIGHT_END_STATE_DDL = 500.0;
        TSHAstar_params.sample.path_qp.DL_LIMIT = 2.0;
        TSHAstar_params.sample.path_qp.VEHICLE_KAPPA_MAX = 0.05;
        TSHAstar_params.sample.path_qp.CENTER_DEVIATION_THRESHOLD = 22;
        TSHAstar_params.sample.path_qp.CENTER_BOUNDS_THRESHOLD = 32;
        TSHAstar_params.sample.path_qp.CENTER_OBS_COEFFICIENT = 10.0;
        planner = new TSHAstar;
        planner->initParams(TSHAstar_params);
    }
    else if (planner_name == "Astar")
    {
        // Astar规划器
        Astar::AstarParams astar_params;
        astar_params.map.OBSTACLE_THRESHOLD = 50;
        astar_params.cost_function.HEURISTICS_TYPE = Astar::HeuristicsType::Euclidean;
        planner = new Astar;
        planner->initParams(astar_params);
    }
    else if (planner_name == "RRT")
    {
        // RRT规划器
        RRT::RRTParams rrt_params;
        rrt_params.map.OBSTACLE_THRESHOLD = 50;
        rrt_params.sample.ITERATOR_TIMES = 10000;
        rrt_params.sample.GOAL_SAMPLE_RATE = 0.05;
        rrt_params.sample.GOAL_DIS_TOLERANCE = 20;
        rrt_params.sample.STEP_SIZE = 40;
        planner = new RRT;
        planner->initParams(rrt_params);
    }
    else if (planner_name == "RRTstar")
    {
        // RRTstar规划器
        RRTstar::RRTstarParams rrtstar_params;
        rrtstar_params.map.OBSTACLE_THRESHOLD = 50;
        rrtstar_params.sample.ITERATOR_TIMES = 10000;
        rrtstar_params.sample.GOAL_SAMPLE_RATE = 0.05;
        rrtstar_params.sample.GOAL_DIS_TOLERANCE = 20;
        rrtstar_params.sample.STEP_SIZE = 40;
        rrtstar_params.sample.NEAR_DIS = 80;
        planner = new RRTstar;
        planner->initParams(rrtstar_params);
    }
    else if (planner_name == "GA")
    {
        // GA规划器
        GA::GAParams ga_params;
        ga_params.map.OBSTACLE_THRESHOLD = 50;
        ga_params.optimization.GENERATION_SIZE = 1000;
        ga_params.optimization.POPULATION_SIZE = 200;
        ga_params.optimization.CHROMOSOME_SIZE = 2;
        ga_params.optimization.CROSSOVER_RATE = 0.7;
        ga_params.optimization.MUTATION_RATE = 0.01;
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
