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

namespace rrt::tree {

template <typename T>
class TreeNode {
public:
    T data;
    std::weak_ptr<TreeNode<T>> parent;
    std::vector<std::shared_ptr<TreeNode<T>>> children;

    TreeNode(const T& data) : data(data) {}
};

template <typename T>
class Tree {
public:
    using NodePtr = std::shared_ptr<TreeNode<T>>;

    Tree() {}

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

private:
    NodePtr root;
    std::vector<NodePtr> nodes;
};

}

#endif //RRT_PLANNER_TREE_H
