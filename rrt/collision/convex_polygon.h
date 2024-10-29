//
// Created by Alex Gisi on 10/8/24.
//
// ConvexPolygon class provides functionality for representing and manipulating convex polygons,
// including collision detection, transformations, and logging.
//
// Reference: Use the Separating Axis Theorem (SAT) for collision detection between convex polygons.
// See https://dyn4j.org/2010/01/sat/ for more information.
//

#ifndef RRT_PLANNER_CONVEX_POLYGON_H
#define RRT_PLANNER_CONVEX_POLYGON_H

#include <vector>
#include <string>
#include <util/vector_2d.h>
#include <util/interval.h>

namespace rrt::collision {

/**
 * @brief Represents a convex polygon in 2D space.
 *
 * The ConvexPolygon class provides methods for creating convex polygons,
 * performing collision detection using the Separating Axis Theorem (SAT),
 * applying geometric transformations, and generating log information.
 */
class ConvexPolygon {
public:
    /**
     * @brief Creates a rectangle as a ConvexPolygon.
     *
     * @param width The width of the rectangle.
     * @param height The height of the rectangle.
     * @param origin The origin point (center) of the rectangle.
     * @param orientation The rotation angle in radians.
     * @return A ConvexPolygon representing the rectangle.
     */
    static ConvexPolygon create_rectangle(double width, double height, const Vector2D& origin = Vector2D(0.0, 0.0), double orientation = 0.0);

    /**
     * @brief Default constructor.
     *
     * Constructs an empty ConvexPolygon.
     */
    ConvexPolygon() = default;

    /**
     * @brief Constructs a ConvexPolygon with the given vertices. Does not
     * verify the vertices form a convex polygon.
     *
     * @param _vertices A vector of vertices defining the convex polygon.
     */
    explicit ConvexPolygon(const std::vector<Vector2D>& _vertices);

    /**
     * @brief Move constructor.
     *
     * Constructs a ConvexPolygon by moving the contents from another ConvexPolygon.
     *
     * @param other The ConvexPolygon to move from.
     */
    ConvexPolygon(ConvexPolygon&& other) noexcept = default;

    /**
     * @brief Move assignment operator.
     *
     * Moves the contents from another ConvexPolygon into this one.
     *
     * @param other The ConvexPolygon to move from.
     * @return A reference to this ConvexPolygon.
     */
    ConvexPolygon& operator=(ConvexPolygon&& other) noexcept = default;

    /**
     * @brief Copy constructor.
     *
     * Constructs a ConvexPolygon by copying another ConvexPolygon.
     *
     * @param other The ConvexPolygon to copy from.
     */
    ConvexPolygon(const ConvexPolygon& other) = default;

    /**
     * @brief Copy assignment operator.
     *
     * Copies the contents from another ConvexPolygon into this one.
     *
     * @param other The ConvexPolygon to copy from.
     * @return A reference to this ConvexPolygon.
     */
    ConvexPolygon& operator=(const ConvexPolygon& other) = default;

    /**
     * @brief Checks for collision with another ConvexPolygon.
     *
     * Uses the Separating Axis Theorem (SAT) to determine if this polygon
     * collides with another polygon.
     *
     * @param other The other ConvexPolygon to check for collision.
     * @return True if the polygons collide; otherwise, false.
     */
    bool collides(const ConvexPolygon& other) const;

    /**
     * @brief Translates the polygon by a given offset.
     *
     * Moves all vertices of the polygon by the specified offset vector.
     *
     * @param offset The vector by which to translate the polygon.
     */
    void translate(const Vector2D& offset);

    /**
     * @brief Rotates the polygon around a given origin.
     *
     * Rotates all vertices of the polygon by the specified angle around the origin point.
     *
     * @param angle_rad The rotation angle in radians.
     * @param origin The point around which to rotate the polygon.
     */
    void rotate(double angle_rad, const Vector2D& origin = Vector2D(0.0, 0.0));

    /**
     * @brief Generates a header string for logging purposes.
     *
     * @return A comma-separated string of vertex coordinate labels.
     */
    std::string log_header() const;

    /**
     * @brief Generates a string representation of the polygon's vertices for logging.
     *
     * @return A comma-separated string of vertex coordinates.
     */
    std::string log() const;

private:
    /**
     * @brief Computes the axes (normals of edges) used in collision detection.
     *
     * @return A vector of axes (normalized perpendicular vectors to the edges).
     */
    std::vector<Vector2D> get_axes() const;

    /**
     * @brief Projects the polygon onto a given axis and computes the projection bounds.
     *
     * Used in the Separating Axis Theorem to find intervals of projections.
     *
     * @param axis The axis onto which to project the polygon.
     * @return A ClosedInterval representing the minimum and maximum projections.
     */
    util::ClosedInterval get_projection_bounds(Vector2D axis) const;

    /// The vertices defining the convex polygon.
    std::vector<Vector2D> vertices;
};

} // namespace rrt::collision

#endif // RRT_PLANNER_CONVEX_POLYGON_H
