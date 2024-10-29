//
// Created by agisi2 on 10/22/2024.
//

#ifndef INTERVAL_H
#define INTERVAL_H

#include <util/util.h>


namespace rrt::util {

class ClosedInterval {
public:
    constexpr ClosedInterval(const double lower, const double upper)
        : lower_(lower), upper_(upper) {}

    constexpr bool intersects(const ClosedInterval &other) const {
        return !(upper_ < other.lower_ || lower_ > other.upper_);
    }

    constexpr bool contains(const double x) const {
        return lower_ <= x && x <= upper_;
    }

    double sample() const {
        return rand(lower_, upper_);
    }

    double lower_;
    double upper_;
};

class OpenInterval {
public:
    constexpr OpenInterval(const double lower, const double upper)
        : lower_(lower), upper_(upper) {}

    constexpr bool intersects(const OpenInterval &other) const {
        return !(upper_ <= other.lower_ || lower_ >= other.upper_);
    }

    constexpr bool contains(const double x) const {
        return lower_ < x && x < upper_;
    }

    double sample() const {
        return rand(lower_, upper_);
    }

    double lower_;
    double upper_;
};

}

#endif //INTERVAL_H
