#include <iostream>
#include <ctime>
#include <filesystem>
#include <models/dubins.h>
#include <util/util.h>
#include <logging/logger.h>

using namespace rrt::models;
using namespace rrt::collision;
using namespace rrt::util;

void dubins_random(const std::filesystem::path &log_dir) {
    Logger<DubinsState> state_logger(log_dir / "state.csv");
    Logger<DubinsCommand> cmd_logger(log_dir / "cmd.csv");
    Logger<DubinsModel> collision_logger(log_dir / "collision_states.csv");

    DubinsModel car(0.5, 1, 1.5);
    const auto obstacle = ConvexPolygon::create_rectangle(0.1, 3, {4.5, 0});

    constexpr int trajectory_n = 100;
    constexpr double dt = 0.05;
    constexpr double trajectory_time = 10;  // seconds
    constexpr int step_n = trajectory_time / dt;

    for (int j = 0; j < trajectory_n; j++) {
        car.set_state(DubinsState());

        for (int i = 0; i < step_n; ++i) {
            auto cmd = DubinsCommand();
            cmd.randomize();

            car.step(cmd, dt);
            if (!car.state().valid()) {
                break;
            }
            state_logger.log(car.state());
            cmd_logger.log(cmd);
        }
        state_logger.increment_episode();
        cmd_logger.increment_episode();
        collision_logger.increment_episode();
    }
}


int main(int argc, char* argv[]) {
    if (argc != 2) {
        std::cout << "One argument required: path to log directory" << std::endl;
        return EXIT_FAILURE;
    }
    const std::filesystem::path log_dir = std::filesystem::absolute(argv[1]);
    rrt::util::create_directory(log_dir);

    dubins_random(log_dir);

    return EXIT_SUCCESS;
}
