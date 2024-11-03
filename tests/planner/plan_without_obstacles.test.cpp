#include <cassert>
#include <chrono>
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
    Logger<Tree<DubinsState>> trajectory_logger(log_dir / "trajectory.csv", false);
    Logger<DubinsModel> planned_states_logger(log_dir / "planned_states.csv", false);
    Logger<Tree<DubinsState>> start_tree_logger(log_dir / "start_tree.csv", false);
    Logger<Tree<DubinsState>> goal_tree_logger(log_dir / "goal_tree.csv", false);
    Logger<std::unordered_map<std::string, DubinsState>> start_goal_logger(log_dir / "start_goal.csv", false);

    // Consider a workspace in [-10, 10] x [-10, 10].
    auto sampler = [&]() -> DubinsState {
        DubinsState sampled;
        sampled.randomize({-10, 10}, {-10, 10});
        return sampled;
    };

    DubinsModel car(0.2, 0.25, 0.75);
    const auto start = DubinsState({0, 0, 0, 0, 0});
    const auto goal = DubinsState({5, -5, M_PI/3, 0, 0});
    const RRTConfiguration<DubinsState> config(3,
                                                0.75,
                                                0.2,
                                                3,
                                                false,
                                                0.1,
                                                0.01,
                                                sampler);

    auto planner = RRTPlanner<DubinsModel, DubinsState, DubinsCommand>(car, config);

    const auto time_start = std::chrono::steady_clock::now();
    const auto trajectory = planner.plan(start, goal, {});
    const auto elapsed = std::chrono::duration<double>(std::chrono::steady_clock::now() - time_start).count();

    const std::unordered_map<std::string, DubinsState> start_goal_map = {
        {"start", start},
        {"goal", goal},
    };

    start_goal_logger.log(start_goal_map);
    start_tree_logger.log(planner.tree_start());
    goal_tree_logger.log(planner.tree_goal());

    assert(trajectory.has_value());

    auto trajectory_tree = Tree<DubinsState>::from_vector(trajectory.value());
    trajectory_logger.log(trajectory_tree);

    for (const auto& state : trajectory.value()) {
        car.set_state(state);
        planned_states_logger.log(car);
    }

    std::cout << "took " << elapsed << "s" << std::endl;
    std::cout << "out of " << config.seconds_limit << "s limit" << std::endl;
    assert(elapsed < config.seconds_limit);
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
