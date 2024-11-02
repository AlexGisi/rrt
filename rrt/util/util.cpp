//
// Created by Alex Gisi on 10/7/24.
//

#include <cmath>
#include <util/util.h>

namespace rrt::util {

double rand(const double a, const double b) {
    std::random_device rd;
    std::mt19937 gen(rd());
    std::uniform_real_distribution dist(a, b);
    return dist(gen);
}

void create_directory(const std::filesystem::path& fp) {
    if (!exists(fp)) {
        if (!std::filesystem::create_directory(fp)) {
            throw std::runtime_error("Could not create directory");
        }
    }
}

// See section 1.2 of "Robot systems" by K. Hauser
double geodesic_distance(const double theta0, const double theta1) {
    if (theta0 < 0 || theta0 >= 2 * M_PI || theta1 < 0 || theta1 >= 2 * M_PI) {
        throw std::runtime_error("Values theta0 ("
            + std::to_string(theta0) + ") or theta1 ("
            + std::to_string(theta1) + ") not in the range [0, 2pi)");
    }

    const double diff = theta1 - theta0;
    double diff_signed;
    if (-M_PI < diff && diff <= M_PI) {
        diff_signed = diff;
    } else if (diff > M_PI) {
        diff_signed = diff - 2*M_PI;
    } else {
        diff_signed = diff + 2*M_PI;
    }

    return std::abs(diff_signed);
}

double geodesic_interpolate(double theta0, double theta1, const double s) {
    // Ensure theta1 and theta2 are in [0, 2*pi)
    constexpr double two_pi = 2 * M_PI;

    theta0 = fmod(theta0, two_pi);
    if (theta0 < 0) theta0 += two_pi;

    theta1 = fmod(theta1, two_pi);
    if (theta1 < 0) theta1 += two_pi;

    // Compute the shortest angular difference
    double delta_theta = theta1 - theta0;
    // Adjust delta_theta to be in (-pi, pi]
    delta_theta = fmod(delta_theta + M_PI, two_pi) - M_PI;

    // Interpolate along the shortest path
    double theta_interp = theta0 + s * delta_theta;

    // Normalize the interpolated angle to [0, 2*pi)
    theta_interp = fmod(theta_interp, two_pi);
    if (theta_interp < 0) theta_interp += two_pi;

    return theta_interp;
}

double wrap_angle(double theta) {
    while (theta > 2*M_PI) {
        theta -= 2*M_PI;
    }
    while (theta < 0) {
        theta += 2*M_PI;
    }
    return theta;
}

std::string append_suffix_to_list(const std::string& input_list, const std::string& suffix) {
    std::stringstream ss(input_list);
    std::string item;
    std::vector<std::string> elements;

    while (std::getline(ss, item, ',')) {
        elements.push_back(item + "_" + suffix);
    }

    std::string result;
    for (size_t i = 0; i < elements.size(); ++i) {
        result += elements[i];
        if (i < elements.size() - 1) {
            result += ",";
        }
    }

    return result;
}

}
