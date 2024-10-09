//
// Created by Alex Gisi on 10/8/24.
// Use the separating axis theorem to determine collision
// between convex polygons.
//
// Ref: https://dyn4j.org/2010/01/sat/
//


#include "convex_polygon.h"

namespace rrt::collision {

 ConvexPolygon ConvexPolygon::create_rectangle(float width,
                                      float height,
                                      const Vector2D& origin,
                                      float orientation) {
    std::vector<Vector2D> vertices = {
            Vector2D(-width / 2.0f, -height / 2.0f),
            Vector2D(width / 2.0f, -height / 2.0f),
            Vector2D(width / 2.0f, height / 2.0f),
            Vector2D(-width / 2.0f, height / 2.0f)
    };

    ConvexPolygon rectangle(vertices);

    if (orientation != 0.0f) {
        rectangle.rotate(orientation);
    }

    if (origin.x() != 0.0f || origin.y() != 0.0f) {
        rectangle.translate(origin);
    }

    return rectangle;
}

bool ConvexPolygon::collides(const ConvexPolygon &other) const {
    auto axes = get_axes();
    auto axes_other = other.get_axes();
    axes.insert(axes.end(), axes_other.begin(), axes_other.end());

    for (auto const &axis : axes) {
        Interval i1 = get_projection_bounds(axis);
        Interval i2 = other.get_projection_bounds(axis);
        if (!i1.overlap(i2)) {
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

Interval ConvexPolygon::get_projection_bounds(Vector2D axis) const {
    float proj_min = axis.dot(vertices[0]);
    float proj_max = proj_min;
    for (auto const &v : vertices) {
        float p = axis.dot(v);
        if (p < proj_min) {
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

void ConvexPolygon::rotate(float angle_rad, const Vector2D& origin) {
    for (auto& vertex : vertices) {
        Vector2D translated = vertex - origin;
        Vector2D rotated = translated.rotated(angle_rad);
        vertex = rotated + origin;
    }
}

std::string ConvexPolygon::log() {
     std::vector<float> vs;
     for (auto const &v : vertices) {
         vs.push_back(v.x());
         vs.push_back(v.y());
     }
 }
}
