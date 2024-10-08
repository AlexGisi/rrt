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

    rrt::planner::RRTConfiguration config(1000, 0.01,
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

int main(int argc, char* argv[]) {
    if (argc != 2) {
        std::cout << "One argument required: path to log directory" << std::endl;
        return EXIT_FAILURE;
    }

    std::filesystem::path log_dir = std::filesystem::absolute(argv[1]);
    rrt::util::create_directory(log_dir);

    q_2a(log_dir);

    return EXIT_SUCCESS;
}
