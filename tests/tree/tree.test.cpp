#include <iostream>
#include <vector>
#include <memory>
#include <cmath>
#include <limits>
#include <cassert>
#include <utility>
#include <tree/tree.h>

using namespace rrt::tree;


double euclidean_distance(const std::pair<double, double>& a, const std::pair<double, double>& b) {
    return std::hypot(a.first - b.first, a.second - b.second);
}

void test_add_nodes() {
    std::cout << "Running test_add_nodes...\n";
    Tree<std::pair<double, double>> tree;
    auto root = tree.add_root({0.0, 0.0});
    assert(root != nullptr);
    assert(tree.get_all_nodes().size() == 1);

    auto node1 = tree.add_node({1.0, 1.0}, root);
    auto node2 = tree.add_node({2.0, 2.0}, node1);
    auto node3 = tree.add_node({3.0, 3.0}, node2);

    assert(tree.get_all_nodes().size() == 4);
    assert(root->children.size() == 1);
    assert(node1->children.size() == 1);
    assert(node2->children.size() == 1);
    assert(node3->children.size() == 0);
    std::cout << "test_add_nodes passed!\n\n";
}

void test_enumerate_nodes() {
    std::cout << "Running test_enumerate_nodes...\n";
    Tree<int> tree;
    auto root = tree.add_root(0);
    tree.add_node(1, root);
    tree.add_node(2, root);

    auto nodes = tree.get_all_nodes();
    assert(nodes.size() == 3);

    std::vector<int> expected_data = {0, 1, 2};
    for (size_t i = 0; i < nodes.size(); ++i) {
        assert(nodes[i]->data == expected_data[i]);
    }
    std::cout << "test_enumerate_nodes passed!\n\n";
}

void test_traverse_to_root() {
    std::cout << "Running test_traverse_to_root...\n";
    Tree<char> tree;
    auto root = tree.add_root('A');
    auto node_b = tree.add_node('B', root);
    auto node_c = tree.add_node('C', node_b);
    auto node_d = tree.add_node('D', node_c);

    auto path = tree.traverse_to_root(node_d);
    std::vector<char> expected_path = {'D', 'C', 'B', 'A'};
    assert(path.size() == expected_path.size());

    for (size_t i = 0; i < path.size(); ++i) {
        assert(path[i]->data == expected_path[i]);
    }
    std::cout << "test_traverse_to_root passed!\n\n";
}

void test_find_nearest() {
    std::cout << "Running test_find_nearest...\n";
    Tree<std::pair<double, double>> tree;
    tree.add_root({0.0, 0.0});
    tree.add_node({1.0, 1.0}, tree.get_all_nodes()[0]);
    tree.add_node({2.0, 2.0}, tree.get_all_nodes()[1]);
    tree.add_node({3.0, 3.0}, tree.get_all_nodes()[2]);

    auto nearest = tree.find_nearest({2.1, 2.1}, euclidean_distance);
    assert(nearest != nullptr);
    assert(nearest->data == std::make_pair(2.0, 2.0));

    nearest = tree.find_nearest({0.5, 0.75}, euclidean_distance);
    assert(nearest->data == std::make_pair(1.0, 1.0));

    std::cout << "test_find_nearest passed!\n\n";
}

int main() {
    test_add_nodes();
    test_enumerate_nodes();
    test_traverse_to_root();
    test_find_nearest();

    std::cout << "All tests passed successfully!\n";
    return 0;
}
