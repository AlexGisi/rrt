#include <iostream>
#include <rrt_collision/convex_polygon.h>
#include <rrt_collision/collision_checker.h>

using namespace rrt::collision;

// Test functions
void test_overlapping_polygons() {
    std::cout << "Running test_overlapping_polygons...\n";

    // Square overlapping another square
    ConvexPolygon square1({{0, 0}, {2, 0}, {2, 2}, {0, 2}});
    ConvexPolygon square2({{1, 1}, {3, 1}, {3, 3}, {1, 3}});

    assert(CollisionChecker::in_collision(square1, square2));
    std::cout << "test_overlapping_polygons passed!\n\n";
}

void test_non_overlapping_polygons() {
    std::cout << "Running test_non_overlapping_polygons...\n";

    // Two squares far apart
    ConvexPolygon square1({{0, 0}, {2, 0}, {2, 2}, {0, 2}});
    ConvexPolygon square2({{5, 5}, {7, 5}, {7, 7}, {5, 7}});

    assert(!CollisionChecker::in_collision(square1, square2));
    std::cout << "test_non_overlapping_polygons passed!\n\n";
}

void test_touching_edges() {
    std::cout << "Running test_touching_edges...\n";

    // Two squares touching at an edge
    ConvexPolygon square1({{0, 0}, {2, 0}, {2, 2}, {0, 2}});
    ConvexPolygon square2({{2, 0}, {4, 0}, {4, 2}, {2, 2}});

    assert(CollisionChecker::in_collision(square1, square2));
    std::cout << "test_touching_edges passed!\n\n";
}

void test_touching_vertices() {
    std::cout << "Running test_touching_vertices...\n";

    // Two squares touching at a vertex
    ConvexPolygon square1({{0, 0}, {2, 0}, {2, 2}, {0, 2}});
    ConvexPolygon square2({{2, 2}, {4, 2}, {4, 4}, {2, 4}});

    assert(CollisionChecker::in_collision(square1, square2));
    std::cout << "test_touching_vertices passed!\n\n";
}

void test_complex_shapes_overlap() {
    std::cout << "Running test_complex_shapes_overlap...\n";

    // Triangle overlapping with a pentagon
    ConvexPolygon triangle({{1, 1}, {3, 1}, {2, 3}});
    ConvexPolygon pentagon({{2, 2}, {4, 2}, {5, 4}, {3, 5}, {1, 4}});

    assert(CollisionChecker::in_collision(triangle, pentagon));
    std::cout << "test_complex_shapes_overlap passed!\n\n";
}

void test_rotated_polygons() {
    std::cout << "Running test_rotated_polygons...\n";

    ConvexPolygon shape_1({{1.5, 1}, {3, 1}, {3, 3}, {1, 3}});
    ConvexPolygon shape_2({{2.5, 1.5}, {1, 4}, {2, 5}});

    assert(CollisionChecker::in_collision(shape_1, shape_2));
    std::cout << "test_rotated_polygons passed!\n\n";
}

void test_close_but_not_touching() {
    std::cout << "Running test_close_but_not_touching...\n";

    // Two squares very close but not touching
    ConvexPolygon square1({{0, 0}, {1, 0}, {1, 1}, {0, 1}});
    ConvexPolygon square2({{1.01, 0}, {2.01, 0}, {2.01, 1}, {1.01, 1}});

    assert(!CollisionChecker::in_collision(square1, square2));
    std::cout << "test_close_but_not_touching passed!\n\n";
}

void test_same_polygon() {
    std::cout << "Running test_same_polygon...\n";

    // A polygon tested against itself
    ConvexPolygon polygon({{0, 0}, {1, 0}, {0.5, 1}});

    assert(CollisionChecker::in_collision(polygon, polygon));
    std::cout << "test_same_polygon passed!\n\n";
}

void test_polygon_inside_another() {
    std::cout << "Running test_polygon_inside_another...\n";

    // Small square inside a larger square
    ConvexPolygon large_square({{0, 0}, {5, 0}, {5, 5}, {0, 5}});
    ConvexPolygon small_square({{2, 2}, {3, 2}, {3, 3}, {2, 3}});

    assert(CollisionChecker::in_collision(large_square, small_square));
    std::cout << "test_polygon_inside_another passed!\n\n";
}

void test_colinear_edges_no_overlap() {
    std::cout << "Running test_colinear_edges_no_overlap...\n";

    // Two rectangles with colinear edges but not overlapping
    ConvexPolygon rect1({{0, 0}, {2, 0}, {2, 1}, {0, 1}});
    ConvexPolygon rect2({{3, 0}, {5, 0}, {5, 1}, {3, 1}});

    assert(!CollisionChecker::in_collision(rect1, rect2));
    std::cout << "test_colinear_edges_no_overlap passed!\n\n";
}

int main() {
    test_overlapping_polygons();
    test_non_overlapping_polygons();
    test_touching_edges();
    test_touching_vertices();
    test_complex_shapes_overlap();
    test_rotated_polygons();
    test_close_but_not_touching();
    test_same_polygon();
    test_polygon_inside_another();
    test_colinear_edges_no_overlap();

    std::cout << "All collision detection tests passed successfully!\n";
    return EXIT_SUCCESS;
}
