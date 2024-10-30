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


void dubins_paths(const std::filesystem::path &log_dir) {
    Logger<DubinsState> state_logger(log_dir / "state.csv");
    Logger<DubinsCommand> cmd_logger(log_dir / "cmd.csv");
    Logger<ConvexPolygon> obstacle_logger(log_dir / "obstacles.csv");
    Logger<DubinsModel> collision_logger(log_dir / "collision_states.csv");

    DubinsModel car(0.5, 1, 1.5);
    const auto obstacle = ConvexPolygon::create_rectangle(0.1, 3, {4.5, 0});

    constexpr int trajectory_n = 55;
    constexpr double dt = 0.05;
    constexpr double trajectory_time = 12;  // seconds
    constexpr int step_n = trajectory_time / dt;

    for (int j = 0; j < trajectory_n; j++) {
        car.set_state(DubinsState());
        obstacle_logger.log(obstacle);

        for (int i = 0; i < step_n; ++i) {
            auto cmd = DubinsCommand();
            cmd.randomize();

            car.step(cmd, dt);
            if (car.in_collision(obstacle)) {
                collision_logger.log(car);
                break;
            }
            if (!car.state().valid()) {
                break;
            }
            state_logger.log(car.state());
            cmd_logger.log(cmd);
        }
        state_logger.increment_episode();
        cmd_logger.increment_episode();
        obstacle_logger.increment_episode();
        collision_logger.increment_episode();
    }
}

int main(int argc, char **argv) {
     if (argc != 2) {
         std::cout << "One argument required: path to log directory" << std::endl;
         return EXIT_FAILURE;
     }
     const std::filesystem::path log_dir = std::filesystem::absolute(argv[1]);
     rrt::util::create_directory(log_dir);

     dubins_paths(log_dir);

    return EXIT_SUCCESS;
}
