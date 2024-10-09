//
// Created by Alex Gisi on 10/3/24.
//

#ifndef RRT_PLANNER_RRT_PLANNER_H
#define RRT_PLANNER_RRT_PLANNER_H

#include <optional>
#include <rrt_tree/tree.h>
#include <rrt_logging/util.h>
#include <rrt_logging/logger.h>
#include <rrt_collision/convex_polygon.h>
#include <rrt_models/dubins_car.h>

namespace rrt::planner {

struct RRTConfiguration {
    int n_iterations;
    std::optional<float> delta;  // Distance from which to connect to goal
    float goal_bias;  // [0, 1]
    int n_cmds_per_iter;
    bool ics_detection;

    float sample_px_lower;
    float sample_px_upper;
    float sample_py_lower;
    float sample_py_upper;
    float sample_dt_lower;
    float sample_dt_upper;

    RRTConfiguration(int _n_iterations,
                     std::optional<float> _delta,
                     float _goal_bias,
                     int _n_commands_per_iter,
                     bool _ics_detection,
                     float _sample_px_lower,
                     float _sample_px_upper,
                     float _sample_py_lower,
                     float _sample_py_upper,
                     float _sample_dt_lower,
                     float _sample_dt_upper) : n_iterations(_n_iterations),
                                               delta(_delta),
                                               goal_bias(_goal_bias),
                                               n_cmds_per_iter(_n_commands_per_iter),
                                               ics_detection(_ics_detection),
                                               sample_px_lower(_sample_px_lower),
                                               sample_px_upper(_sample_px_upper),
                                               sample_py_lower(_sample_py_lower),
                                               sample_py_upper(_sample_py_upper),
                                               sample_dt_lower(_sample_dt_lower),
                                               sample_dt_upper(_sample_dt_upper) {}
};


template <typename ModelT, typename StateT, typename CommandT>
class RRTPlanner {
public:
    RRTPlanner(ModelT _model, StateT _start, StateT _goal)
        : start(_start),
          goal(_goal),
          model(_model) {
        root = tree.add_root(start);
    };

    bool plan(RRTConfiguration config, Logger<ModelT>* collision_logger = nullptr, const std::vector<collision::ConvexPolygon>& obstacles = {}) {
        for (int i = 0; i < config.n_iterations; ++i) {
            StateT q_samp;
            if (rrt::util::rand(0, 1) < config.goal_bias) {
                q_samp = goal;
            } else {
                do { // Sample from free space.
                    q_samp = StateT::sample(
                        config.sample_px_lower,
                        config.sample_px_upper,
                        config.sample_py_lower,
                        config.sample_py_upper);
                    model.set_state(q_samp);
                } while (model.in_collision(obstacles));

            }
            auto node_nearest = tree.find_nearest(q_samp, StateT::weighted_distance);

            // Simulate some commands, see which gets the best state.
            StateT state_best;
            float dist_best = std::numeric_limits<float>::max();;
            float dt = rrt::util::rand(config.sample_dt_lower,config.sample_dt_upper);
            for (int j = 0; j < config.n_cmds_per_iter; ++j) {
                model.set_state(node_nearest->data);
                auto cmd = CommandT::sample();
                model.step(cmd, dt);
                auto q_new = model.get_state();

                float dist = StateT::weighted_distance(q_new, goal);
                if (dist < dist_best) {
                    state_best = q_new;
                    dist_best = dist;
                }
            }

            // Check inevitable collision
            if (config.ics_detection) {
                StateT after_brake = state_best.after_brake();
                model.set_state(after_brake);
                if (model.in_collision(obstacles)) {
                    continue;
                }
            }

            // Rough collision checking: just check final and a point halfway in between
            model.set_state(state_best);
            if (model.in_collision(obstacles)) {
                if (collision_logger) {
                    collision_logger->log(model);
                }
                continue;
            }
            auto halfway = StateT::interpolate(node_nearest->data, state_best, 0.5);
            model.set_state(halfway);
            if (model.in_collision(obstacles)) {
                continue;
            }

            if (!state_best.violates_constraints()) {
                auto node_new = tree.add_node(state_best, node_nearest);

                // TODO: final edge does not respect kinodynamic constraint...
                if (config.delta && StateT::weighted_distance(state_best, goal) < config.delta.value()) {
                    tree.add_node(goal, node_new);
                    return true;
                }
            }
        }
        return false;
    }

    StateT start;
    StateT goal;
    rrt::tree::Tree<StateT> tree;
    typename rrt::tree::Tree<StateT>::NodePtr root;
    ModelT model;
};
}


#endif //RRT_PLANNER_RRT_PLANNER_H
