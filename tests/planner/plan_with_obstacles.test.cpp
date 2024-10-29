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
    Logger<ConvexPolygon> obstacle_logger(log_dir / "obstacles.csv");
    Logger<DubinsModel> collision_logger(log_dir / "collision_states.csv");

    DubinsModel car(0.5, 1.0, 1.5);

    const auto rect1 = ConvexPolygon::create_rectangle(rrt::util::rand(2.0, 6.0), rrt::util::rand(0.5, 3.0), {rrt::util::rand(-8, -2), rrt::util::rand(-4, -8)});
    const auto rect2 = ConvexPolygon::create_rectangle(rrt::util::rand(0.5, 2.0), rrt::util::rand(2.5, 5.0), {rrt::util::rand(2, 10), rrt::util::rand(-8, 8)});
    const auto rect3 = ConvexPolygon::create_rectangle(rrt::util::rand(0.5, 2.0), rrt::util::rand(0.5, 3.0), {rrt::util::rand(-8, 8), rrt::util::rand(3, 9)});
    std::vector obstacles = {rect1, rect2, rect3};

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
    auto goal = DubinsState({0, 0, 0, 0, 0});

    auto planner = RRTPlanner<DubinsModel, DubinsState, DubinsCommand>(car, config);
    const bool success = planner.plan(start, goal, obstacles);
    assert(success == false);

    state_logger.log(planner.tree());
    for (auto const& obstacle : obstacles) {
        obstacle_logger.log(obstacle);
    }
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
