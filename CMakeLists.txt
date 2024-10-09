cmake_minimum_required(VERSION 3.10)
project(rrt_planner)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(xtl REQUIRED)
find_package(xtensor REQUIRED)

include_directories(${CMAKE_SOURCE_DIR}/rrt)

set(SOURCE_FILES
        rrt/rrt_models/dubins_car.h
        rrt/rrt_models/dubins_car.cpp
        rrt/rrt_collision/collision_checker.h
        rrt/rrt_collision/collision_checker.cpp
        rrt/rrt_tree/tree.h
        rrt/rrt_tree/tree.cpp
        rrt/rrt_planner/rrt_planner.h
        rrt/rrt_planner/rrt_planner.cpp
        rrt/rrt_logging/logger.h
        rrt/rrt_logging/logger.cpp
        rrt/rrt_logging/util.h
        rrt/rrt_logging/util.cpp
        rrt/rrt_collision/convex_polygon.cpp
        rrt/rrt_collision/convex_polygon.h
        rrt/rrt_collision/vector_2d.h)

set(INCLUDE_DIRECTORIES
        rrt/rrt_models
        rrt/rrt_collision
        rrt/rrt_tree
        rrt/rrt_planner
        rrt/rrt_logging)

add_library(rrt_lib ${SOURCE_FILES})
target_include_directories(rrt_lib PUBLIC ${xtensor_INCLUDE_DIRS})
target_link_libraries(rrt_lib PUBLIC xtensor)

function(add_rrt_test TEST_NAME TEST_SOURCE)
    add_executable(${TEST_NAME} ${TEST_SOURCE} ${SOURCE_FILES})
    target_link_libraries(${TEST_NAME} PRIVATE rrt_lib)
    target_include_directories(${TEST_NAME} PRIVATE ${INCLUDE_DIRECTORIES})
endfunction()

add_rrt_test(dubins_car_test tests/models/dubins_car.test.cpp)
add_rrt_test(collision_checker_test tests/collision/collision_checker.test.cpp)
add_rrt_test(rrt_planner_test tests/planner/rrt_planner.test.cpp)
add_rrt_test(tree_test tests/tree/tree.test.cpp)