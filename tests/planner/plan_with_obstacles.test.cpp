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


void plan_with_obstacles(const std::filesystem::path &log_dir) {
    Logger<Tree<DubinsState>> state_logger(log_dir / "tree.csv", false);
    Logger<ConvexPolygon> obstacle_logger(log_dir / "obstacles.csv", false);
    Logger<DubinsModel> planned_states_logger(log_dir / "planned_states.csv", false);
    Logger<std::unordered_map<std::string, DubinsState>> start_goal_logger(log_dir / "start_goal.csv", false);

    const DubinsModel car(0.2, 0.25, 0.75);

    const auto rect1 = ConvexPolygon::create_rectangle(-0.1, 8, {-2, 0});
    // const auto rect2 = ConvexPolygon::create_rectangle(rrt::util::rand(0.5, 2.0), rrt::util::rand(2.5, 5.0), {rrt::util::rand(2, 10), rrt::util::rand(-8, 8)});
    // const auto rect3 = ConvexPolygon::create_rectangle(rrt::util::rand(0.5, 2.0), rrt::util::rand(0.5, 3.0), {rrt::util::rand(-8, 8), rrt::util::rand(3, 9)});
    std::vector obstacles = {rect1};

    // Consider a workspace in [-10, 10] x [-10, 10].
    auto sampler = [&](DubinsState* state) {
        state->randomize({-10, 10}, {-10, 10});
    };
    RRTConfiguration<DubinsState> config(1000,
        0.01,
        0,
    1,
    false,
    0.1,
    0.01,
    sampler);
    auto start = DubinsState({0, 0, 0, 0, 0});
    auto goal = DubinsState({5, 5, M_PI/3, 0, 0});

    auto planner = RRTPlanner<DubinsModel, DubinsState, DubinsCommand>(car, config);
    const bool success = planner.plan(start, goal, obstacles);

    const std::unordered_map<std::string, DubinsState> start_goal_map = {
        {"start", start},
        {"goal", goal},
    };

    start_goal_logger.log(start_goal_map);

    state_logger.log(planner.tree());
    for (auto const& obstacle : obstacles) {
        obstacle_logger.log(obstacle);
    }

    assert(success == true);
}

int main(int argc, char **argv) {
    if (argc != 2) {
        std::cout << "One argument required: path to log directory" << std::endl;
        return EXIT_FAILURE;
    }
    const std::filesystem::path log_dir = std::filesystem::absolute(argv[1]);
    rrt::util::create_directory(log_dir);

    plan_with_obstacles(log_dir);

    return EXIT_SUCCESS;
}
