#include <cassert>
#include <iostream>
#include <filesystem>
#include <util/util.h>
#include <collision/convex_polygon.h>

using namespace rrt::collision;


void test_overlapping_polygons() {
    std::cout << "Running test_overlapping_polygons...\n";

    // Square overlapping another square
    const ConvexPolygon square1({{0, 0}, {2, 0}, {2, 2}, {0, 2}});
    const ConvexPolygon square2({{1, 1}, {3, 1}, {3, 3}, {1, 3}});

    assert(square1.collides(square2));
    std::cout << "test_overlapping_polygons passed!\n\n";
}

void test_non_overlapping_polygons() {
    std::cout << "Running test_non_overlapping_polygons...\n";

    // Two squares far apart
    const ConvexPolygon square1({{0, 0}, {2, 0}, {2, 2}, {0, 2}});
    const ConvexPolygon square2({{5, 5}, {7, 5}, {7, 7}, {5, 7}});

    assert(!square1.collides(square2));
    std::cout << "test_non_overlapping_polygons passed!\n\n";
}

void test_touching_edges() {
    std::cout << "Running test_touching_edges...\n";

    // Two squares touching at an edge
    const ConvexPolygon square1({{0, 0}, {2, 0}, {2, 2}, {0, 2}});
    const ConvexPolygon square2({{2, 0}, {4, 0}, {4, 2}, {2, 2}});

    assert(square1.collides(square2));
    std::cout << "test_touching_edges passed!\n\n";
}

void test_touching_vertices() {
    std::cout << "Running test_touching_vertices...\n";

    // Two squares touching at a vertex
    const ConvexPolygon square1({{0, 0}, {2, 0}, {2, 2}, {0, 2}});
    const ConvexPolygon square2({{2, 2}, {4, 2}, {4, 4}, {2, 4}});

    assert(square1.collides(square2));
    std::cout << "test_touching_vertices passed!\n\n";
}

void test_complex_shapes_overlap() {
    std::cout << "Running test_complex_shapes_overlap...\n";

    // Triangle overlapping with a pentagon
    const ConvexPolygon triangle({{1, 1}, {3, 1}, {2, 3}});
    const ConvexPolygon pentagon({{2, 2}, {4, 2}, {5, 4}, {3, 5}, {1, 4}});

    assert(pentagon.collides(triangle));
    std::cout << "test_complex_shapes_overlap passed!\n\n";
}

void test_rotated_polygons() {
    std::cout << "Running test_rotated_polygons...\n";

    const ConvexPolygon shape_1({{1.5, 1}, {3, 1}, {3, 3}, {1, 3}});
    const ConvexPolygon shape_2({{2.5, 1.5}, {1, 4}, {2, 5}});

    assert(shape_1.collides(shape_2));
    std::cout << "test_rotated_polygons passed!\n\n";
}

void test_close_but_not_touching() {
    std::cout << "Running test_close_but_not_touching...\n";

    // Two squares very close but not touching
    const ConvexPolygon square1({{0, 0}, {1, 0}, {1, 1}, {0, 1}});
    const ConvexPolygon square2({{1.01, 0}, {2.01, 0}, {2.01, 1}, {1.01, 1}});

    assert(!square1.collides(square2));
    std::cout << "test_close_but_not_touching passed!\n\n";
}

void test_same_polygon() {
    std::cout << "Running test_same_polygon...\n";

    // A polygon tested against itself
    const ConvexPolygon polygon({{0, 0}, {1, 0}, {0.5, 1}});

    assert(polygon.collides(polygon));
    std::cout << "test_same_polygon passed!\n\n";
}

void test_polygon_inside_another() {
    std::cout << "Running test_polygon_inside_another...\n";

    // Small square inside a larger square
    const ConvexPolygon large_square({{0, 0}, {5, 0}, {5, 5}, {0, 5}});
    const ConvexPolygon small_square({{2, 2}, {3, 2}, {3, 3}, {2, 3}});

    assert(small_square.collides(large_square));
    std::cout << "test_polygon_inside_another passed!\n\n";
}

void test_colinear_edges_no_overlap() {
    std::cout << "Running test_colinear_edges_no_overlap...\n";

    // Two rectangles with colinear edges but not overlapping
    const ConvexPolygon rect1({{0, 0}, {2, 0}, {2, 1}, {0, 1}});
    const ConvexPolygon rect2({{3, 0}, {5, 0}, {5, 1}, {3, 1}});

    assert(!rect1.collides(rect2));
    std::cout << "test_colinear_edges_no_overlap passed!\n\n";
}

void collision_test_suite() {
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
}

int main(int argc, char **argv) {
    collision_test_suite();
    return EXIT_SUCCESS;
}
