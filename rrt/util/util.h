//
// Created by Alex Gisi on 10/7/24.
//

#pragma once

#include <string>
#include <sstream>
#include <vector>
#include <random>
#include <algorithm>
#include <filesystem>

namespace rrt::util {

double rand(double a, double b);
void create_directory(const std::filesystem::path& fp);
double geodesic_distance(double theta0, double theta1);
double geodesic_interpolate(double theta0, double theta1, double s);
double wrap_angle(double theta);
std::string append_suffix_to_list(const std::string& input_list, const std::string& suffix);

template<typename T>
std::vector<T> shuffle_vector(std::vector<T> vec) {
    auto rd = std::random_device {};
    auto rng = std::default_random_engine { rd() };
    std::shuffle(std::begin(vec), std::end(vec), rng);
    return vec;
}

template <typename Container>
std::string comma_join(const Container& values) {
    std::ostringstream buffer;
    auto it = values.begin();
    auto end = values.end();

    if (it != end) {
        buffer << *it;
        ++it;
    }

    for (; it != end; ++it) {
        buffer << ',' << *it;
    }

    return buffer.str();
}

template<typename T>
std::vector<T> arange(T start, T stop, T step = 1) {
    std::vector<T> values;
    for (T value = start; value < stop; value += step)
        values.push_back(value);
    return values;
}

template <typename T>
std::vector<T> reverse_vector(const std::vector<T>& input) {
    std::vector<T> reversed(input.rbegin(), input.rend());
    return reversed;
}

template <typename T>
std::vector<T> concatenate_vectors(const std::vector<T>& vec1, const std::vector<T>& vec2) {
    std::vector<T> result;
    result.reserve(vec1.size() + vec2.size());  // Reserve space for both vectors
    result.insert(result.end(), vec1.begin(), vec1.end());  // Insert first vector
    result.insert(result.end(), vec2.begin(), vec2.end());  // Insert second vector
    return result;
}
}
