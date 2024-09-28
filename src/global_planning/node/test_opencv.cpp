#include <ros/ros.h>
#include <ros/package.h>

#include "global_planning/astar.h"
#include "global_planning/MCAstar.h"
#include "global_planning/map_generator.h"


int main(int argc, char * argv[])
{
    ros::init(argc, argv, "test_opencv");
    ros::NodeHandle nh;

    // Astar规划器
    Astar * astar_planner = new Astar;
    Astar::AstarParams astar_params;
    astar_params.map_params.OBSTACLE_THRESHOLD = 50;
    astar_params.cost_function_params.HEURISTICS_TYPE = Astar::HeuristicsType::Euclidean;
    astar_planner->initParams(astar_params);

    // MCAstar规划器
    MCAstar * MCAstar_planner = new MCAstar;
    MCAstar::MCAstarParams MCAstar_params;
    MCAstar_params.map_params.EXPANDED_K = 1;
    MCAstar_params.map_params.EXPANDED_MIN_THRESHOLD = 0;
    MCAstar_params.map_params.EXPANDED_MAX_THRESHOLD = 100;
    MCAstar_params.map_params.COST_THRESHOLD = 10;
    MCAstar_params.map_params.OBSTACLE_THRESHOLD = 100;
    MCAstar_params.cost_function_params.HEURISTICS_TYPE = MCAstar::HeuristicsType::Euclidean;
    MCAstar_params.cost_function_params.TRAV_COST_K = 2.0;
    MCAstar_params.bezier_curve_params.T_STEP = 0.01;
    MCAstar_params.downsampling_params.INTERVAL = 0.3;
    MCAstar_planner->initParams(MCAstar_params);

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
    // generator.set_planner(astar_planner);
    generator.set_planner(MCAstar_planner);
    generator.show_map("test", 10);

    delete astar_planner;
    delete MCAstar_planner;
    return 0;
}
