#include <cassert>
#include <iostream>
#include <filesystem>
#include <filesystem>
#include <xtensor/xarray.hpp>
#include <xtensor/xrandom.hpp>
#include <rrt_models/dubins_car.h>
#include <rrt_logging/util.h>
#include <rrt_logging/logger.h>
#include <rrt_collision/convex_polygon.h>
#include <rrt_collision/collision_checker.h>
#include <rrt_planner/rrt_planner.h>

using namespace rrt::collision;
using namespace rrt::models;
using namespace rrt::planner;
using namespace rrt::tree;

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

void q_3a(const std::filesystem::path &log_dir) {
    Logger<rrt::models::DubinsState> state_logger(log_dir / "state.csv");
    Logger<rrt::models::DubinsCommand> cmd_logger(log_dir / "cmd.csv");
    Logger<rrt::collision::ConvexPolygon> obstacle_logger(log_dir / "obstacles.csv");
    Logger<rrt::models::DubinsCar> car_logger(log_dir / "collision_states.csv");

    rrt::models::DubinsCar car(0.5);
    auto rect = rrt::collision::ConvexPolygon::create_rectangle(0.1, 1, {1, 0});

    int trajectories = 20;
    int steps = 200;
    float time_traj = 10;  // seconds
    float dt = time_traj / steps;

    for (int j = 0; j < trajectories; j++) {
        car.reset();
        obstacle_logger.log(rect);

        auto t = xt::linspace<float>(0, time_traj, steps);
        float c = rrt::util::rand(0, 2*M_PI);
        float d = rrt::util::rand(0, 2*M_PI);

        auto a = 10 * xt::sin(10*t + c);
        auto psi = 3 * xt::sin(4*t + d);

        for (int i = 0; i < t.size(); ++i) {
            auto cmd = rrt::models::DubinsCommand(a(i), psi(i));
            car.step(cmd, dt);

            if (car.in_collision({rect})) {
                car_logger.log(car);
                break;
            }
            state_logger.log(car.get_state());
            cmd_logger.log(cmd);
        }
        state_logger.increment_episode();
        cmd_logger.increment_episode();
        obstacle_logger.increment_episode();
        car_logger.increment_episode();
    }
}

void q_3b(const std::filesystem::path &log_dir) {
    Logger<Tree<DubinsState>> state_logger(log_dir / "tree.csv", false);
    Logger<ConvexPolygon> obstacle_logger(log_dir / "obstacles.csv");
    Logger<DubinsCar> collision_logger(log_dir / "collision_states.csv");

    DubinsCar car(0.5);

    auto rect1 = ConvexPolygon::create_rectangle(rrt::util::rand(2.0, 6.0), rrt::util::rand(0.5, 3.0), {rrt::util::rand(-8, -2), rrt::util::rand(-4, -8)});
    auto rect2 = ConvexPolygon::create_rectangle(rrt::util::rand(0.5, 2.0), rrt::util::rand(2.5, 5.0), {rrt::util::rand(2, 10), rrt::util::rand(-8, 8)});
    auto rect3 = ConvexPolygon::create_rectangle(rrt::util::rand(0.5, 2.0), rrt::util::rand(0.5, 3.0), {rrt::util::rand(-8, 8), rrt::util::rand(3, 9)});
    std::vector<ConvexPolygon> obstacles = {rect1, rect2, rect3};

    rrt::planner::RRTConfiguration config(1000, std::nullopt,
                                          0, 1, false,
                                          -10, 10,
                                          -10, 10,
                                          0, 1.0);
    DubinsState start = {0, 0, 0, 0, 0};
    DubinsState goal = {0, 0, 0, 0, 0};

    auto planner = RRTPlanner<DubinsCar, DubinsState, DubinsCommand>(car, start, goal);
    bool success = planner.plan(config, &collision_logger, obstacles);
    assert(success == false);

    state_logger.log(planner.tree);
    for (auto const& obstacle : obstacles) {
        obstacle_logger.log(obstacle);
    }
}

void q_3c(const std::filesystem::path &log_dir) {
    Logger<Tree<DubinsState>> state_logger(log_dir / "tree.csv", false);
    Logger<ConvexPolygon> obstacle_logger(log_dir / "obstacles.csv");
    Logger<DubinsCar> collision_logger(log_dir / "collision_states.csv");

    DubinsCar car(0.5);

    auto rect1 = ConvexPolygon::create_rectangle(rrt::util::rand(2.0, 6.0), rrt::util::rand(0.5, 3.0), {rrt::util::rand(-8, -2), rrt::util::rand(-4, -8)});
    auto rect2 = ConvexPolygon::create_rectangle(rrt::util::rand(0.5, 2.0), rrt::util::rand(2.5, 5.0), {rrt::util::rand(2, 10), rrt::util::rand(-8, 8)});
    auto rect3 = ConvexPolygon::create_rectangle(rrt::util::rand(0.5, 2.0), rrt::util::rand(0.5, 3.0), {rrt::util::rand(-8, 8), rrt::util::rand(3, 9)});
    std::vector<ConvexPolygon> obstacles = {rect1, rect2, rect3};

    rrt::planner::RRTConfiguration config(1000, std::nullopt,
                                          0, 1, true,
                                          -10, 10,
                                          -10, 10,
                                          0, 1.0);
    DubinsState start = {0, 0, 0, 0, 0};
    DubinsState goal = {0, 0, 0, 0, 0};

    auto planner = RRTPlanner<DubinsCar, DubinsState, DubinsCommand>(car, start, goal);
    bool success = planner.plan(config, &collision_logger, obstacles);
    assert(success == false);

    state_logger.log(planner.tree);
    for (auto const& obstacle : obstacles) {
        obstacle_logger.log(obstacle);
    }
}

int main(int argc, char **argv) {
    if (argc != 2) {
        std::cout << "One argument required: path to log directory" << std::endl;
        return EXIT_FAILURE;
    }
    xt::random::seed(time(nullptr));

    std::filesystem::path log_dir = std::filesystem::absolute(argv[1]);
    rrt::util::create_directory(log_dir);

    q_3c(log_dir);

    return EXIT_SUCCESS;
}
