# rrt
A ground-up implementation of the [RRT](https://en.wikipedia.org/wiki/Rapidly_exploring_random_tree) algorithm which respects kinodynamic constraints.

## Install
There are no dependencies. Download the source and build with cmake.

## Use
Presently the implemented model in `rrt::models` is the `DubinsModel`. The `dubins_car_test` performs planning and logs the results to CSV. The [rrt-plot](https://github.com/AlexGisi/rrt-plot) library can be used to visualize the search and resulting path.

## Features
- Kinodynamic RRT (`rrt::planner`)
- Dubins car dynamic model (`rrt::models`)
- Support for adding any other model by subclassing from `rrt::models::Model`, `rrt::models::State`, and `rrt::models::Command`
- CSV logging for arbitrary objects which implement `log` and `log_header` methods
- Tree data structure (`rrt::tree`)
- Collision checking betweeen convex polygons (`rrt::collision`)
