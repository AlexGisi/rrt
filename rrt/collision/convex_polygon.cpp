//
// Created by Alex Gisi on 10/8/24.
// Use the separating axis theorem to determine collision
// between convex polygons.
//
// Ref: https://dyn4j.org/2010/01/sat/
//

#include <array>
#include <collision/convex_polygon.h>
#include <util/util.h>

namespace rrt::collision {

 ConvexPolygon ConvexPolygon::create_rectangle(const double width,
                                              const double height,
                                              const Vector2D& origin,
                                              const double orientation) {
     const std::vector vertices = {
            Vector2D(-width / 2.0, -height / 2.0),
            Vector2D(width / 2.0, -height / 2.0),
            Vector2D(width / 2.0, height / 2.0),
            Vector2D(-width / 2.0, height / 2.0)
    };

    ConvexPolygon rectangle(vertices);

    if (orientation != 0.0) {
        rectangle.rotate(orientation);
    }

    if (origin.x() != 0.0 || origin.y() != 0.0) {
        rectangle.translate(origin);
    }

    return rectangle;
}

ConvexPolygon::ConvexPolygon(const std::vector<Vector2D> &_vertices) : vertices(_vertices) {}

bool ConvexPolygon::collides(const ConvexPolygon &other) const {
    auto axes = get_axes();
    auto axes_other = other.get_axes();
    axes.insert(axes.end(), axes_other.begin(), axes_other.end());

    for (auto const &axis : axes) {
        util::ClosedInterval i1 = get_projection_bounds(axis);
        util::ClosedInterval i2 = other.get_projection_bounds(axis);
        if (!i1.intersects(i2)) {
            return false;
        }
    }
    return true;
}

std::vector<Vector2D> ConvexPolygon::get_axes() const {
    std::vector<Vector2D> axes;
    for (int i = 0; i < vertices.size(); ++i) {
        auto v1 = vertices[i];
        auto v2 = vertices[i == vertices.size()-1 ? 0 : i+1];
        auto edge = v2 - v1;
        axes.push_back(edge.perpendicular());
    }

    return axes;
}

util::ClosedInterval ConvexPolygon::get_projection_bounds(const Vector2D axis) const {
    double proj_min = axis.dot(vertices[0]);
    double proj_max = proj_min;
    for (auto const &v : vertices) {
        if (const double p = axis.dot(v); p < proj_min) {
            proj_min = p;
        } else if (p > proj_max) {
            proj_max = p;
        }
    }
    return {proj_min, proj_max};
}

void ConvexPolygon::translate(const Vector2D& offset) {
    for (auto& vertex : vertices) {
        vertex += offset;
    }
}

void ConvexPolygon::rotate(const double angle_rad, const Vector2D& origin) {
    for (auto& vertex : vertices) {
        Vector2D translated = vertex - origin;
        Vector2D rotated = translated.rotated(angle_rad);
        vertex = rotated + origin;
    }
}

std::string ConvexPolygon::log_header() const {
    std::string s;
    for(int i = 0; i < vertices.size(); ++i) {
        s += "x" + std::to_string(i) + ',' + "y" + std::to_string(i) + ',';
    }
     return s;
}

std::string ConvexPolygon::log() const {
     std::vector<double> vs;
     for (auto const &v : vertices) {
         vs.push_back(v.x());
         vs.push_back(v.y());
     }
     return util::comma_join<std::vector<double>>(vs);
 }
}
