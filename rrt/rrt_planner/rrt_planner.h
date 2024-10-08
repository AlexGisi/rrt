//
// Created by Alex Gisi on 10/3/24.
//
// TODO
// - make the sample space configurable
//

#ifndef RRT_PLANNER_RRT_PLANNER_H
#define RRT_PLANNER_RRT_PLANNER_H

#include <rrt_tree/tree.h>
#include <rrt_logging/util.h>

namespace rrt::planner {

struct RRTConfiguration {
    int n_iterations;
    float delta;  // Distance from which to connect to goal

    float sample_px_lower;
    float sample_px_upper;
    float sample_py_lower;
    float sample_py_upper;
    float sample_dt_lower;
    float sample_dt_upper;

    RRTConfiguration(int _n_iterations,
                     float _delta,
                     float _sample_px_lower,
                     float _sample_px_upper,
                     float _sample_py_lower,
                     float _sample_py_upper,
                     float _sample_dt_lower,
                     float _sample_dt_upper) :
                         n_iterations(_n_iterations),
                         delta(_delta),
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
        : model(_model),
          start(_start),
          goal(_goal) {
        root = tree.add_root(start);
    };

    bool plan(RRTConfiguration config) {
        for (int i = 0; i < config.n_iterations; ++i) {
            auto q_samp = StateT::sample(
                    config.sample_px_lower,
                    config.sample_px_upper,
                    config.sample_py_lower,
                    config.sample_py_upper);
            auto node_nearest = tree.find_nearest(q_samp, StateT::distance);

            model.set_state(node_nearest->data);
            auto cmd = CommandT::sample();
            float dt = rrt::util::rand(config.sample_dt_lower,
                                       config.sample_dt_upper);
            model.step(cmd, dt);
            auto q_new = model.get_state();
            if (!q_new.violates_constraints()) {
                auto node_new = tree.add_node(q_new, node_nearest);

                if (StateT::distance(q_new, goal) < config.delta) {
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
