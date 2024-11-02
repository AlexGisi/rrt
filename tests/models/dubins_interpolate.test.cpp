//
// Created by agisi2 on 11/2/2024.
//

#include <iostream>
#include <filesystem>
#include <models/dubins.h>
#include <logging/logger.h>

using namespace rrt::models;

void dubins_interpolate(const std::filesystem::path& log_dir) {
    Logger<DubinsState> state_logger(log_dir / "state.csv");

    const DubinsState start_state({0, 0, 0.3, 0, 0});
    const DubinsState end_state({0, 2, 6.1, 0, 0});

    std::vector<DubinsState> states;
    for(double i = 0; i < 1; i = i + 0.1) {
        states.push_back(start_state.interpolate(end_state, i));
        state_logger.log(states.back());
    }
}

int main(int argc, char* argv[]) {
    if (argc != 2) {
        std::cout << "One argument required: path to log directory" << std::endl;
        return EXIT_FAILURE;
    }
    const std::filesystem::path log_dir = std::filesystem::absolute(argv[1]);
    rrt::util::create_directory(log_dir);

    dubins_interpolate(log_dir);

    return EXIT_SUCCESS;
}
