#include <filesystem>
#include <rrt_planner/rrt_planner.h>
#include <rrt_tree/tree.h>
#include <rrt_models/dubins_car.h>
#include <rrt_logging/logger.h>

using namespace rrt::models;
using namespace rrt::planner;
using namespace rrt::tree;

void q_2a(const std::filesystem::path& log_dir) {
    Logger<Tree<DubinsState>> logger(log_dir / "tree.csv", false);

    rrt::planner::RRTConfiguration config(1000, std::nullopt,
                                          0, 1, false,
                                          -10, 10,
                                          -10, 10,
                                          0, 1.0);
    DubinsState start = {0, 0, 0, 0, 0};
    DubinsState goal = {100, 100, 0, 0, 0};
    DubinsCar car(0.5);

    auto planner = RRTPlanner<DubinsCar, DubinsState, DubinsCommand>(car, start, goal);
    bool success = planner.plan(config);
    assert(success == false);

    logger.log(planner.tree);
}

void q_2c(const std::filesystem::path& log_dir) {
    Logger<Tree<DubinsState>> logger(log_dir / "tree.csv", false);

    rrt::planner::RRTConfiguration config(1000, std::nullopt,
                                          0.9, 1, false,
                                          -10, 10,
                                          -10, 10,
                                          0, 1.0);
    DubinsState start = {0, 0, 0, 0, 0};
    DubinsState goal = {3, 4, M_PI/2, 0, 0};
    DubinsCar car(0.5);

    auto planner = RRTPlanner<DubinsCar, DubinsState, DubinsCommand>(car, start, goal);
    bool success = planner.plan(config);
    assert(success == false);

    logger.log(planner.tree);
    auto q_nearest = planner.tree.find_nearest(goal, DubinsState::euclidean_distance)->data;
    float dist = DubinsState::euclidean_distance(q_nearest, goal);
    std::cout << dist << std::endl;
}

void q_2d(const std::filesystem::path& log_dir) {
    Logger<Tree<DubinsState>> logger(log_dir / "tree.csv", false);

    rrt::planner::RRTConfiguration config(1000, std::nullopt,
                                          0.5, 3, false,
                                          -10, 10,
                                          -10, 10,
                                          0, 1.0);
    DubinsState start = {0, 0, 0, 0, 0};
    DubinsState goal = {3, 4, M_PI/2, 0, 0};
    DubinsCar car(0.5);

    auto planner = RRTPlanner<DubinsCar, DubinsState, DubinsCommand>(car, start, goal);
    bool success = planner.plan(config);
    assert(success == false);

    logger.log(planner.tree);
    auto q_nearest = planner.tree.find_nearest(goal, DubinsState::euclidean_distance)->data;
    float dist = DubinsState::euclidean_distance(q_nearest, goal);
    std::cout << dist << std::endl;
}

int main(int argc, char* argv[]) {
    if (argc != 2) {
        std::cout << "One argument required: path to log directory" << std::endl;
        return EXIT_FAILURE;
    }

    std::filesystem::path log_dir = std::filesystem::absolute(argv[1]);
    rrt::util::create_directory(log_dir);

    q_2d(log_dir);

    return EXIT_SUCCESS;
}
