//
// Created by Alex Gisi on 10/8/24.
//

#ifndef RRT_PLANNER_CONVEX_POLYGON_H
#define RRT_PLANNER_CONVEX_POLYGON_H

#include <vector>
#include <string>
#include <rrt_collision/vector_2d.h>

namespace rrt::collision {
class Interval {
public:
    Interval() : lower(0.0), upper(0.0) {}
    Interval(float _lower, float _upper) : lower(_lower), upper(_upper) {}

    bool overlap(const Interval &other) const {
        return !(upper < other.lower || lower > other.upper);
    }

    float lower;
    float upper;
};

class ConvexPolygon {
public:
    static ConvexPolygon create_rectangle(float width, float height, const Vector2D& origin = Vector2D(0.0, 0.0), float orientation = 0.0);

    ConvexPolygon() = default;
    explicit ConvexPolygon(const std::vector<Vector2D>& _vertices) : vertices(_vertices) {}

    bool collides(const ConvexPolygon& other) const;
    std::vector<Vector2D> get_axes() const;
    Interval get_projection_bounds(Vector2D axis) const;

    void translate(const Vector2D& offset);
    void rotate(float angle_rad, const Vector2D& origin = Vector2D(0.0, 0.0));

    std::string log_header();
    std::string log() const;
private:
    std::vector<Vector2D> vertices;
};
}

#endif //RRT_PLANNER_CONVEX_POLYGON_H
