#pragma once

#include <vector>
#include <memory>

#include "point.hpp"

namespace RRT {
    struct Node {
        Node* parent;
        std::vector<std::unique_ptr<Node>> children;
        Point point;
        double cost;

        Node(Node* parent, Point point)
            : parent(parent), children(), point(point), cost(0)
        {}

        Node* getParentRaw() const;

        double calculateCost();
        void propagateCost();
    };
}