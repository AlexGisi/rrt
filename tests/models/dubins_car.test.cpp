#include <iostream>
#include <ctime>
#include <filesystem>
#include <xtensor/xarray.hpp>
#include <xtensor/xrandom.hpp>
#include <rrt_models/dubins_car.h>
#include <rrt_logging/util.h>
#include <rrt_logging/logger.h>

void q_1b(const std::filesystem::path& log_dir) {
    Logger<rrt::models::DubinsState> state_logger(log_dir / "state.csv");
    rrt::models::DubinsCar car(0.5);

    int n_traj = 100;
    int steps = 200;
    float time_traj = 10;  // seconds
    float dt = time_traj / steps;

    for (int i = 0; i < n_traj; ++i) {
        car.reset();
        for (int s = 0; s < steps; ++s) {
            auto cmd = rrt::models::DubinsCommand::sample();
            car.step(cmd, dt);
            state_logger.log(car.get_state());

            if (car.get_state().violates_constraints()) {
                break;
            }
        }
        state_logger.increment_episode();
    }
}

void q_1c(const std::filesystem::path &log_dir) {
    Logger<rrt::models::DubinsState> state_logger(log_dir / "state.csv");
    Logger<rrt::models::DubinsCommand> cmd_logger(log_dir / "cmd.csv");
    rrt::models::DubinsCar car(0.5);

    int steps = 200;
    float time_traj = 10;  // seconds
    float dt = time_traj / steps;

    auto t = xt::linspace<float>(0, time_traj, steps);
    float c = rrt::util::rand(0, 2*M_PI);
    float d = rrt::util::rand(0, 2*M_PI);

    auto a = 10 * xt::sin(10*t + c);
    auto psi = 3 * xt::sin(4*t + d);

    for (int i = 0; i < t.size(); ++i) {
        auto cmd = rrt::models::DubinsCommand(a(i), psi(i));
        car.step(cmd, dt);

        state_logger.log(car.get_state());
        cmd_logger.log(cmd);
    }
}

int main(int argc, char* argv[]) {
    if (argc != 2) {
        std::cout << "One argument required: path to log directory" << std::endl;
        return EXIT_FAILURE;
    }
    xt::random::seed(time(nullptr));

    std::filesystem::path log_dir = std::filesystem::absolute(argv[1]);
    rrt::util::create_directory(log_dir);

    q_1c(log_dir);

    return EXIT_SUCCESS;
}
