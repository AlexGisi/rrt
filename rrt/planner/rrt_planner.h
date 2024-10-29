//
// Created by Alex Gisi on 10/3/24.
//

#ifndef RRT_PLANNER_RRT_PLANNER_H
#define RRT_PLANNER_RRT_PLANNER_H

#include <optional>
#include <tree/tree.h>
#include <util/util.h>
#include <logging/logger.h>
#include <collision/convex_polygon.h>
#include <models/dubins.h>

namespace rrt::planner {

template <typename StateT>
struct RRTConfiguration {
    int iter_max_n;
    double tol;
    double goal_bias;  // [0, 1]
    int n_cmds_per_iter;
    bool ics_detection;
    double sample_dt;
    double line_check_resolution;
    std::function<void(StateT*)> sampler;

    RRTConfiguration(const int iter_max_n,
                     const double tol,
                     const double goal_bias,
                     const int n_commands_per_iter,
                     const bool ics_detection,
                     const double sample_dt,
                     const double line_check_resolution,
                     std::function<void(StateT*)> sampler)
        :  iter_max_n(iter_max_n),
           tol(tol),
           goal_bias(goal_bias),
           n_cmds_per_iter(n_commands_per_iter),
           ics_detection(ics_detection),
           sample_dt(sample_dt),
           line_check_resolution(line_check_resolution),
           sampler(sampler) {}
};


template <typename ModelT, typename StateT, typename CommandT>
class RRTPlanner {
public:
    RRTPlanner(ModelT model, const RRTConfiguration<StateT> &configuration)
        : model_(model), config_(configuration) {
        tree_ = tree::Tree<StateT>();
    }

    bool plan(StateT start, StateT goal, const std::vector<collision::ConvexPolygon>& obstacles = {}) {
        auto root = tree_.add_root(start);

        for (int i = 0; i < config_.iter_max_n; ++i) {
            StateT state_sampled;
            if (util::rand(0, 1) < config_.goal_bias) {
                state_sampled = goal;
            } else {
                do {
                    state_sampled.sample(config_.sampler);
                    model_.set_state(state_sampled);
                } while (model_.in_collision(obstacles));  // Sample from free space.

            }
            auto dist = [](StateT a, StateT b) -> double { return a.distance(b); };
            auto node_nearest = tree_.find_nearest(state_sampled, dist);

            // Simulate some commands from the nearest node, see which gets
            // closest to the sampled state.
            StateT state_best;
            double dist_best = std::numeric_limits<double>::max();;
            for (int j = 0; j < config_.n_cmds_per_iter; ++j) {
                model_.set_state(node_nearest->data);
                auto cmd = CommandT();
                cmd.randomize();
                model_.step(cmd, config_.sample_dt);
                auto state_new = model_.state();

                if (const double dist = state_new.distance(state_sampled); dist < dist_best) {
                    state_best = state_new;
                    dist_best = dist;
                }
            }

            // Line collision checking. Check the interpolated points between
            // sampled and new state at the resolution defined by the config.
            auto ss  = util::arange<double>(0, 1, config_.line_check_resolution / dist_best);
            bool collides = false;
            for (auto const &s : ss ) {
                auto state_interp = node_nearest->data.interpolate(state_best, s);
                model_.set_state(state_interp);
                if (model_.in_collision(obstacles)) {
                    collides = true;
                    break;
                }
            }
            if (collides) { continue; }

            model_.set_state(state_best);
            if (model_.in_collision(obstacles)) {
                continue;
            }

            if (!state_best.valid()) {
                auto node_new = tree_.add_node(state_best, node_nearest);

                if (state_best.distance(goal) < config_.tol) {
                    return true;
                }
            }
        }
        return false;
    }

    tree::Tree<StateT> tree() { return tree_; }

private:
    tree::Tree<StateT> tree_;
    ModelT model_;
    RRTConfiguration<StateT> config_;
};
}


#endif //RRT_PLANNER_RRT_PLANNER_H
