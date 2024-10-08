//
// Created by Alex Gisi on 10/7/24.
//

#include <ctime>
#include <rrt_logging/util.h>

namespace rrt::util {

float rand(float a, float b) {
    std::random_device rd;
    std::mt19937 gen(rd());
    std::uniform_real_distribution<float> dist(a, b);
    return dist(gen);
}

void create_directory(const std::filesystem::path& fp) {
    if (!std::filesystem::exists(fp)) {
        if (!std::filesystem::create_directory(fp)) {
            throw std::runtime_error("Could not create directory");
        }
    }
}

}
