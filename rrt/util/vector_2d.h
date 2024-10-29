//
// Created by Alex Gisi on 10/8/24.
//

#ifndef RRT_PLANNER_VECTOR_2D_H
#define RRT_PLANNER_VECTOR_2D_H

#include <cmath>

namespace rrt::collision {
class Vector2D {
public:
    constexpr Vector2D() : x_(0.0), y_(0.0) {}
    constexpr Vector2D(const double x, const double y) : x_(x), y_(y) {}

    constexpr double x() const { return x_; }
    constexpr double y() const { return y_; }

    // Operator overloads
    constexpr Vector2D operator+(const Vector2D &other) const {
        return {x_ + other.x_, y_ + other.y_};
    }

    constexpr Vector2D operator-(const Vector2D &other) const {
        return {x_ - other.x_, y_ - other.y_};
    }

    constexpr Vector2D operator-() const {
        return {-x_, -y_};
    }

    // Compound assignment operators
    Vector2D &operator+=(const Vector2D &other) {
        x_ += other.x_;
        y_ += other.y_;
        return *this;
    }

    Vector2D &operator-=(const Vector2D &other) {
        x_ -= other.x_;
        y_ -= other.y_;
        return *this;
    }

    // Scalar multiplication and division
    constexpr Vector2D operator*(const double scalar) const {
        return {x_ * scalar, y_ * scalar};
    }

    constexpr Vector2D operator/(const double scalar) const {
        return {x_ / scalar, y_ / scalar};
    }

    Vector2D &operator*=(const double scalar) {
        x_ *= scalar;
        y_ *= scalar;
        return *this;
    }

    Vector2D &operator/=(const double scalar) {
        x_ /= scalar;
        y_ /= scalar;
        return *this;
    }

    // Equality operators
    constexpr bool operator==(const Vector2D &other) const {
        return x_ == other.x_ && y_ == other.y_;
    }

    constexpr bool operator!=(const Vector2D &other) const {
        return !(*this == other);
    }

    // Misc
    constexpr double dot(const Vector2D &other) const {
        return x_ * other.x_ + y_ * other.y_;
    }

    constexpr Vector2D perpendicular() const {
        return {-y_, x_};
    }

    double magnitude() const {
        return std::hypot(x_, y_);
    }

    constexpr double magnitude_squared() const {
        return x_ * x_ + y_ * y_;
    }

    Vector2D normalized() const {
        if (const double mag = magnitude(); mag > 0.0) {
            return {x_ / mag, y_ / mag};
        }
        return {0.0, 0.0};
    }

    // Rotate the vector by an angle (in radians)
    Vector2D rotated(const double angle_rad) const {
        const double cos_a = std::cos(angle_rad);
        const double sin_a = std::sin(angle_rad);
        return {x_ * cos_a - y_ * sin_a, x_ * sin_a + y_ * cos_a};
    }

private:
    double x_;
    double y_;
};

}

#endif //RRT_PLANNER_VECTOR_2D_H
