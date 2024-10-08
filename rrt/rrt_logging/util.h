//
// Created by Alex Gisi on 10/7/24.
//

#include <xtensor/xarray.hpp>
#include <string>
#include <sstream>
#include <vector>
#include <random>
#include <filesystem>
#include <exception>

#pragma once

namespace rrt::util {

float rand(float a, float b);
void create_directory(const std::filesystem::path& fp);
float geodesic_distance(float theta0, float theta1);
float wrap_angle(float theta);

template<typename T>
std::string comma_join(const std::vector<T> &values) {
    std::ostringstream buffer;
    for (auto it = values.begin(); it != values.end(); ++it) {
        buffer << *it;
        if (std::next(it) != values.end()) {
            buffer << ',';
        }
    }
    return buffer.str();
}
}
