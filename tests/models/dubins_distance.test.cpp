#include <cassert>
#include <chrono>
#include <iostream>
#include <filesystem>
#include <logging/logger.h>
#include <models/dubins.h>


using namespace rrt::models;


void dubins_distance(const std::filesystem::path &log_dir) {
    Logger<DubinsModel> state_logger(log_dir / "state.csv", false);

    const auto s0 = DubinsState({0, 0, 0, 0, 0});
    const auto s1 = DubinsState({0.1, 0.1, 0.5, 0.7, 0.1});
    std::cout << s0.distance(s1) << std::endl;

    DubinsModel m(0.8, 1.2, 2.1);
    m.set_state(s0);
    state_logger.log(m);

    m.set_state(s1);
    state_logger.log(m);
}

int main(int argc, char **argv) {
    if (argc != 2) {
        std::cout << "One argument required: path to log directory" << std::endl;
        return EXIT_FAILURE;
    }
    const std::filesystem::path log_dir = std::filesystem::absolute(argv[1]);
    rrt::util::create_directory(log_dir);

    dubins_distance(log_dir);

    return EXIT_SUCCESS;
}
