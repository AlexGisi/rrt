cmake_minimum_required(VERSION 3.10)
project(rrt_planner)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

include_directories(${CMAKE_SOURCE_DIR}/rrt)

set(SOURCE_FILES
        rrt/tree/tree.h
        rrt/planner/rrt_planner.h
        rrt/logging/logger.h
        rrt/util/util.h
        rrt/util/util.cpp
        rrt/collision/convex_polygon.cpp
        rrt/collision/convex_polygon.h
        rrt/util/vector_2d.h
        rrt/util/interval.h
        rrt/models/model.h
        rrt/models/dubins.h
        tests/models/dubins_interpolate.test.cpp)

set(INCLUDE_DIRECTORIES
    rrt/models
    rrt/collision
    rrt/tree
    rrt/planner
    rrt/logging
    rrt/rrt_util)

add_library(rrt_lib ${SOURCE_FILES})

function(add_rrt_test TEST_NAME TEST_SOURCE)
    add_executable(${TEST_NAME} ${TEST_SOURCE} ${SOURCE_FILES})
    target_link_libraries(${TEST_NAME} PRIVATE rrt_lib)
    target_include_directories(${TEST_NAME} PRIVATE ${INCLUDE_DIRECTORIES})
endfunction()

# --- Tests ---
# Models
add_rrt_test(dubins_random_test tests/models/dubins_random.test.cpp)
add_rrt_test(dubins_interpolate_test tests/models/dubins_interpolate.test.cpp)

# Collisions
add_rrt_test(collision_checker_test tests/collision/collision_checker.test.cpp)
add_rrt_test(dubins_paths_test tests/collision/dubins_paths.test.cpp)

# Planner
add_rrt_test(plan_with_obstacles_test tests/planner/plan_with_obstacles.test.cpp)
add_rrt_test(plan_without_obstacles_test tests/planner/plan_without_obstacles.test.cpp)

# Tree
add_rrt_test(tree_test tests/tree/tree.test.cpp)

# Util
add_rrt_test(interval_test tests/util/interval.test.cpp)
