#pragma once

#include <vector>
#include <memory>

#include <Eigen/Dense>

namespace rrt {
    struct Node {
        Node* parent;
        std::vector<std::unique_ptr<Node>> children;
        Eigen::Vector2d point;
        double cost;
        double cumulativeDistance;
        double angleWeight;

        Node(Node* parent, Eigen::Vector2d point, double angleWeight=0)
            : parent(parent), children(), point(point), cost(0), angleWeight(angleWeight), cumulativeDistance(0)
        {}

        double calculateCost();
        void propagateCost();
    };
}