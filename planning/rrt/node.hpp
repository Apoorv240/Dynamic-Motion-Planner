#pragma once

#include <vector>
#include <memory>

#include "../../math/Vec2d.hpp"

namespace RRT {
    struct Node {
        Node* parent;
        std::vector<std::unique_ptr<Node>> children;
        Vec2d point;
        double cost;

        Node(Node* parent, Vec2d point)
            : parent(parent), children(), point(point), cost(0)
        {}

        Node* getParentRaw() const;

        double calculateCost();
        void propagateCost();
    };
}