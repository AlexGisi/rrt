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
    constexpr Vector2D(float x, float y) : x_(x), y_(y) {}

    constexpr float x() const { return x_; }
    constexpr float y() const { return y_; }

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
    constexpr Vector2D operator*(float scalar) const {
        return {x_ * scalar, y_ * scalar};
    }

    constexpr Vector2D operator/(float scalar) const {
        return {x_ / scalar, y_ / scalar};
    }

    Vector2D &operator*=(float scalar) {
        x_ *= scalar;
        y_ *= scalar;
        return *this;
    }

    Vector2D &operator/=(float scalar) {
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
    constexpr float dot(const Vector2D &other) const {
        return x_ * other.x_ + y_ * other.y_;
    }

    constexpr Vector2D perpendicular() const {
        return {-y_, x_};
    }

    float magnitude() const {
        return std::hypot(x_, y_);
    }

    constexpr float magnitude_squared() const {
        return x_ * x_ + y_ * y_;
    }

    Vector2D normalized() const {
        float mag = magnitude();
        if (mag > 0.0) {
            return {x_ / mag, y_ / mag};
        }
        return {0.0, 0.0};
    }

    // Rotate the vector by an angle (in radians)
    Vector2D rotated(float angle_rad) const {
        float cos_a = std::cos(angle_rad);
        float sin_a = std::sin(angle_rad);
        return {x_ * cos_a - y_ * sin_a, x_ * sin_a + y_ * cos_a};
    }

private:
    float x_;
    float y_;
};

}

#endif //RRT_PLANNER_VECTOR_2D_H
