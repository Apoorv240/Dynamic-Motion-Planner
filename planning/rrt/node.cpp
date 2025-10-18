#include "node.hpp"

using namespace RRT;

double Node::calculateCost() {
    if (parent == nullptr) {
        return 0;
    }
    double newCost = parent->cost + (point - parent->point).magnitude();//point.distToPoint(parent->point);
    cost = newCost;
    return newCost;
}

void Node::propagateCost() {
    for (auto &child : children) {
        child->calculateCost();
        child->propagateCost();
    }
}