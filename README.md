# rrt
A ground-up implementation of the [RRT](https://en.wikipedia.org/wiki/Rapidly_exploring_random_tree) algorithm which respects kinodynamic constraints.

## Install
The only dependency is [xtensor](https://xtensor.readthedocs.io/en/latest/installation.html) and [xtl](https://github.com/xtensor-stack/xtl), install these from source with `CMAKE_INSTALL_PREFIX=/usr`.

## Use
Presently the implemented model in `rrt::models` is the `DubinsCar`. The `dubins_car_test` performs planning and logs the results to CSV. The [rrt-plot](https://github.com/AlexGisi/rrt-plot) library can be used to visualize the search and resulting path.

## Features
- Kinodynamic RRT (`rrt::planner`)
- Dubins car dynamic model (`rrt::models`)
- CSV logging for arbitrary objects which implement `log` and `log_header` methods
- Tree data structure (`rrt::tree`)
- Collision checking betweeen convex polygons (`rrt::collision`)
