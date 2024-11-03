//
// Created by Alex Gisi on 10/3/24.
//

#ifndef RRT_PLANNER_RRT_PLANNER_H
#define RRT_PLANNER_RRT_PLANNER_H

#include <algorithm>
#include <chrono>
#include <optional>
#include <tree/tree.h>
#include <util/util.h>
#include <collision/convex_polygon.h>

namespace rrt::planner {

template <typename StateT>
struct RRTConfiguration {
    double seconds_limit;
    double tol;
    double goal_bias;  // [0, 1]
    int n_cmds_per_iter;
    bool ics_detection;
    double sample_dt;
    double line_check_resolution;
    std::function<StateT(void)> state_space_sampler;

    RRTConfiguration(const double seconds_limit,
                     const double tol,
                     const double goal_bias,
                     const int n_commands_per_iter,
                     const bool ics_detection,
                     const double sample_dt,
                     const double line_check_resolution,
                     std::function<StateT(void)> state_space_sampler)
        :
           seconds_limit(seconds_limit),
           tol(tol),
           goal_bias(goal_bias),
           n_cmds_per_iter(n_commands_per_iter),
           ics_detection(ics_detection),
           sample_dt(sample_dt),
           line_check_resolution(line_check_resolution),
           state_space_sampler(state_space_sampler) {}
};


template <typename ModelT, typename StateT, typename CommandT>
class RRTPlanner {
public:
    RRTPlanner(ModelT model, const RRTConfiguration<StateT> &configuration)
        : model_(model), config_(configuration) {

        tree_start_ = tree::Tree<StateT>();
        tree_goal_ = tree::Tree<StateT>();
    }

    void prepare_search(StateT start, StateT goal) {
        tree_start_.clear();
        tree_goal_.clear();
        tree_start_.add_root(start);
        tree_goal_.add_root(goal);
    }

    std::optional<std::vector<StateT>> plan(StateT start, StateT goal, const std::vector<collision::ConvexPolygon>& obstacles = {}) {
        if (!is_state_viable(start, obstacles) || !is_state_viable(goal, obstacles)) {
            return std::nullopt;
        }

        std::vector<StateT> trajectory;
        prepare_search(start, goal);

        const auto start_time = std::chrono::steady_clock::now();
        while (std::chrono::duration<double>(std::chrono::steady_clock::now() - start_time).count() < config_.seconds_limit) {
            auto start_tree_extension = extend_tree(tree_start_, goal, obstacles);
            auto goal_tree_extension = extend_tree(tree_goal_, start, obstacles);

            if (start_tree_extension) {
                const auto nearest_in_goal_tree = tree_goal_.find_nearest(start_tree_extension->data, [](StateT a, StateT b) -> double { return a.distance(b); });
                if (nearest_in_goal_tree->data.distance(start_tree_extension->data) < config_.tol) {
                    auto extension_to_start = tree_start_.traverse_to_root(start_tree_extension);
                    auto nearest_to_goal = tree_goal_.traverse_to_root(nearest_in_goal_tree);

                    // Get the states from the vector of TreeNode pointers.
                    std::vector<StateT> extension_to_start_states;
                    std::vector<StateT> nearest_to_goal_states;
                    auto deref = [](const auto& ptr) {
                        return ptr->data;
                    };
                    std::transform(extension_to_start.begin(), extension_to_start.end(), std::back_inserter(extension_to_start_states), deref);
                    std::transform(nearest_to_goal.begin(), nearest_to_goal.end(), std::back_inserter(nearest_to_goal_states), deref);

                    trajectory = util::concatenate_vectors(util::reverse_vector(extension_to_start_states), nearest_to_goal_states);
                    break;
                }
            }

            if (goal_tree_extension) {
                const auto nearest_in_start_tree = tree_start_.find_nearest(goal_tree_extension->data, [](StateT a, StateT b) -> double { return a.distance(b); });
                if (nearest_in_start_tree->data.distance(goal_tree_extension->data) < config_.tol) {
                    auto nearest_to_start = tree_start_.traverse_to_root(nearest_in_start_tree);
                    auto extension_to_goal = tree_goal_.traverse_to_root(goal_tree_extension);

                    // Get list of states.
                    std::vector<StateT> nearest_to_start_states;
                    std::vector<StateT> extension_to_goal_states;
                    auto deref = [](const auto& ptr) {
                        return ptr->data;
                    };
                    std::transform(nearest_to_start.begin(), nearest_to_start.end(), std::back_inserter(nearest_to_start_states), deref);
                    std::transform(extension_to_goal.begin(), extension_to_goal.end(), std::back_inserter(extension_to_goal_states), deref);

                    trajectory = util::concatenate_vectors(util::reverse_vector(nearest_to_start_states), extension_to_goal_states);
                    break;
                }
            }

        }

        return trajectory.empty() ? std::nullopt : std::optional(trajectory);
    }

    bool is_state_viable(const StateT &state, const std::vector<collision::ConvexPolygon>& obstacles = {}) {
        model_.set_state(state);
        return !model_.in_collision(obstacles) && state.valid();
    }

    std::shared_ptr<tree::TreeNode<StateT>> extend_tree(tree::Tree<StateT> &tree, StateT bias_target, const std::vector<collision::ConvexPolygon>& obstacles = {}) {
        StateT state_sampled = (util::rand(0, 1) < config_.goal_bias) ? bias_target : config_.state_space_sampler();
        model_.set_state(state_sampled);
        if (model_.in_collision(obstacles)) {
            return nullptr;
        }

        auto node_nearest = tree.find_nearest(state_sampled, [](StateT a, StateT b) -> double { return a.distance(b); });
        auto state_nearest = node_nearest->data;

        // Simulate some commands from the nearest node, see which gets
        // closest to the sampled state and extend to that one.
        StateT state_extension;
        double dist_sample_to_extension = std::numeric_limits<double>::max();;
        for (int j = 0; j < config_.n_cmds_per_iter; ++j) {
            model_.set_state(node_nearest->data);
            auto cmd = CommandT();
            cmd.randomize();
            model_.step(cmd, config_.sample_dt);
            auto state_candidate = model_.state();

            if (const double dist = state_candidate.distance(state_sampled); dist < dist_sample_to_extension) {
                state_extension = state_candidate;
                dist_sample_to_extension = dist;
            }
        }

        // Line collision checking. Check the interpolated points between
        // sampled and new state at the resolution defined by the config.
        double dist_to_best =  state_nearest.distance(state_extension);
        auto ss  = util::arange<double>(0, 1, config_.line_check_resolution / dist_to_best);
        ss.push_back(1);  // arange doesn't include endpoint.
        bool collides = false;
        for (auto const &s : ss ) {
            if (const auto state_interp = state_nearest.interpolate(state_extension, s); !is_state_viable(state_interp, obstacles)) {
                collides = true;
                break;
            }
        }
        if (collides) { return nullptr; }

        auto node_new = tree.add_node(state_extension, node_nearest);
        return node_new;
    }

    tree::Tree<StateT> tree_start() { return tree_start_; }
    tree::Tree<StateT> tree_goal() { return tree_goal_; }

private:
    tree::Tree<StateT> tree_start_;
    tree::Tree<StateT> tree_goal_;

    ModelT model_;
    RRTConfiguration<StateT> config_;
};
}


#endif //RRT_PLANNER_RRT_PLANNER_H
