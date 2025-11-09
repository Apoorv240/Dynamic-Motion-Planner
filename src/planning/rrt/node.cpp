#include "planning/rrt/node.hpp"

using namespace rrt;

#include <iostream>

double Node::calculateCost() {
    if (parent == nullptr) {
        return 0;
    }

    Eigen::Vector2d v1, v2;
    if (parent->parent == nullptr) {
        v1 = Eigen::Vector2d(0,0);
        v2 = Eigen::Vector2d(0,0);
    } else {
        v1 = parent->point - parent->parent->point;
        v2 = point - parent->point;
    }

    double angleCost = 0.0;
    if (v1.norm() > 1e-6 && v2.norm() > 1e-6) {
        double cosTheta = v1.dot(v2) / (v1.norm() * v2.norm());
        if (cosTheta > 1.0) cosTheta = 1.0;
        if (cosTheta < -1.0) cosTheta = -1.0;
        angleCost = angleWeight * (1 - cosTheta);
    }

    cumulativeDistance = parent->cumulativeDistance + (point - parent->point).norm();

    double newCost = angleCost + parent->cost + (point - parent->point).norm();
    cost = newCost;
    return newCost;
}

void Node::propagateCost() {
    for (auto &child : children) {
        child->calculateCost();
        child->propagateCost();
    }
}