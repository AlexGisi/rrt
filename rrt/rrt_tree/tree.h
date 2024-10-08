//
// Created by Alex Gisi on 10/3/24.
//

#ifndef RRT_PLANNER_TREE_H
#define RRT_PLANNER_TREE_H

#include <iostream>
#include <vector>
#include <memory>
#include <unordered_map>
#include <stack>
#include <rrt_logging/util.h>

namespace rrt::tree {

template <typename Loggable>
class TreeNode {
public:
    Loggable data;
    std::weak_ptr<TreeNode<Loggable>> parent;
    std::vector<std::shared_ptr<TreeNode<Loggable>>> children;

    explicit TreeNode(const Loggable& data) : data(data) {}
};

template <typename T>
class Tree {
public:
    using NodePtr = std::shared_ptr<TreeNode<T>>;

    Tree() = default;

    NodePtr add_root(const T& data) {
        root = std::make_shared<TreeNode<T>>(data);
        nodes.push_back(root);
        return root;
    }

    NodePtr add_node(const T& data, NodePtr parent) {
        auto node = std::make_shared<TreeNode<T>>(data);
        node->parent = parent;
        parent->children.push_back(node);
        nodes.push_back(node);
        return node;
    }

    const std::vector<NodePtr>& get_all_nodes() const {
        return nodes;
    }

    std::vector<NodePtr> get_all_leafs() const {
        std::vector<NodePtr> leafs;
        for (auto n : nodes) {
            if (n->children.empty()) {
                leafs.push_back(n);
            }
        }
        return leafs;
    }

    std::vector<NodePtr> traverse_to_root(NodePtr leaf) const {
        std::vector<NodePtr> path;
        auto current = leaf;
        while (current) {
            path.push_back(current);
            current = current->parent.lock();
        }
        return path;
    }

    template <typename DistanceFunc>
    NodePtr find_nearest(const T& point, DistanceFunc distance_func) const {
        NodePtr nearest = nullptr;
        double min_distance = std::numeric_limits<double>::max();

        for (const auto& node : nodes) {
            double distance = distance_func(node->data, point);
            if (distance < min_distance) {
                min_distance = distance;
                nearest = node;
            }
        }
        return nearest;
    }

    std::string log_header() {
        std::string data_header = T::log_header();
        std::string header = "branch," + data_header;
        return header;
    }

    std::string log() {
        std::string log_str;
        auto leafs = get_all_leafs();
        for (auto it = leafs.begin(); it != leafs.end(); ++it) {
            auto branch_nodes = traverse_to_root(*it);
            int branch = it - leafs.begin();
            for (auto n : branch_nodes) {
                log_str += std::to_string(branch) + ',';
                log_str += n->data.log() + '\n';
            }
        }
        return log_str;
    }

private:
    NodePtr root;
    std::vector<NodePtr> nodes;
};

}

#endif //RRT_PLANNER_TREE_H
