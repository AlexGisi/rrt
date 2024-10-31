#include <cassert>
#include <iostream>
#include <filesystem>
#include <models/dubins.h>
#include <util/util.h>
#include <logging/logger.h>
#include <collision/convex_polygon.h>
#include <planner/rrt_planner.h>

using namespace rrt::collision;
using namespace rrt::models;
using namespace rrt::planner;
using namespace rrt::tree;


void plan_without_obstacles(const std::filesystem::path &log_dir) {
    Logger<Tree<DubinsState>> state_logger(log_dir / "tree.csv", false);
    Logger<DubinsModel> planned_states_logger(log_dir / "planned_states.csv", false);
    Logger<std::unordered_map<std::string, DubinsState>> start_goal_logger(log_dir / "start_goal.csv", false);

    const DubinsModel car(0.2, 0.25, 0.75);

    // Consider a workspace in [-10, 10] x [-10, 10].
    auto sampler = [&](DubinsState* state) {
        state->randomize({-10, 10}, {-10, 10});
    };
    const RRTConfiguration<DubinsState> config(1000,
        0.01,
        0.2,
    3,
    false,
    0.1,
    0.01,
    sampler);
    const auto start = DubinsState({0, 0, 0, 0, 0});
    const auto goal = DubinsState({5, -5, M_PI/3, 0, 0});

    auto planner = RRTPlanner<DubinsModel, DubinsState, DubinsCommand>(car, config);
    const bool success = planner.plan(start, goal, {});

    const std::unordered_map<std::string, DubinsState> start_goal_map = {
        {"start", start},
        {"goal", goal},
    };

    start_goal_logger.log(start_goal_map);
    state_logger.log(planner.tree());
    assert(success == true);

    // todo: planned states
}

int main(int argc, char **argv) {
    if (argc != 2) {
        std::cout << "One argument required: path to log directory" << std::endl;
        return EXIT_FAILURE;
    }
    const std::filesystem::path log_dir = std::filesystem::absolute(argv[1]);
    rrt::util::create_directory(log_dir);

    plan_without_obstacles(log_dir);

    return EXIT_SUCCESS;
}
