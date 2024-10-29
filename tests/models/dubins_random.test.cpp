#include <iostream>
#include <ctime>
#include <filesystem>
#include <models/dubins.h>
#include <util/util.h>
#include <logging/logger.h>

using namespace rrt::models;
using namespace rrt::collision;
using namespace rrt::util;

void dubins_random(const std::filesystem::path& log_dir) {
    Logger<DubinsState> state_logger(log_dir / "state.csv");
    DubinsModel car(0.5, 1.5, 1.0);

    constexpr int n_traj = 100;
    constexpr int steps = 200;
    constexpr float time_traj = 10;  // seconds
    constexpr float dt = time_traj / steps;

    for (int i = 0; i < n_traj; ++i) {
        car.set_state(DubinsState());
        for (int s = 0; s < steps; ++s) {
            auto cmd = DubinsCommand();
            cmd.randomize();

            car.step(cmd, dt);
            state_logger.log(car.state());

            if (!car.state().valid()) {
                break;
            }
        }
        state_logger.increment_episode();
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
