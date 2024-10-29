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

double wrap_angle(double theta) {
    while (theta > 2*M_PI) {
        theta -= 2*M_PI;
    }
    while (theta < 0) {
        theta += 2*M_PI;
    }
    return theta;
}

}
