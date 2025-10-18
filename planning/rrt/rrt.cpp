#include "rrt.hpp"
#include <algorithm>

using namespace RRT;

Point Generator::genRandPoint() const {
    std::uniform_real_distribution<> bias(0, 1);
    if (bias(randomNumberGenerator) < goalBias) {
        return goal;
    }
    std::uniform_real_distribution<> distX(bounds.minX, bounds.maxX);
    std::uniform_real_distribution<> distY(bounds.minY, bounds.maxY);
    return Point(distX(randomNumberGenerator), distY(randomNumberGenerator));
}

Node* Generator::nearestNode(const Point& point) const {
    return nodeManager.nearestNeighbor(point);
}

void Generator::nodesInRadiusofPoint(std::vector<Node*>& nodeList, double radius, const Point& point) const {
    nodeManager.radiusSearch(nodeList, radius, point);
}

Node* Generator::findBestParent(const std::vector<Node*>& nodeList, const Point& point, Node* nearestNode) const {
    Node* bestParent = nearestNode;
    double bestCost = bestParent->cost + point.distToPoint(bestParent->point);

    for (const auto& node : nodeList) {
        double cost = node->cost + point.distToPoint(node->point);
        if (cost < bestCost) {
            bestParent = node;
            bestCost = cost;
        }
    }

    return bestParent;
}

bool Generator::pointIsValid(const Point& p) const {
    for (const auto& obstacle : obstacles) {
        if (obstacle.pointInObstacle(p)) {
            return false;
        }
    }
    return true;
}

void Generator::iterate() {
    Point randPoint = genRandPoint();

    // Nearest Node to Point
    Node* nearestNode = this->nearestNode(randPoint);

    // Normalize the point to the node
    if (randPoint == nearestNode->point) {
        return;
    }
    randPoint.normalizeDistToPoint(stepSize, nearestNode->point);

    // Check Collision
    if (!pointIsValid(randPoint)) {
        return;
    }

    // Choose best parent
    double searchRadius = std::max(stepSize * 2, 
        stepSize * std::sqrt(std::log(nodeManager.size) / nodeManager.size));
    std::vector<Node*> nearbyNodes;
    nodesInRadiusofPoint(nearbyNodes, searchRadius, randPoint);

    Node* bestParent = findBestParent(nearbyNodes, randPoint, nearestNode);

    // Wire new node to the best parent
    std::unique_ptr<Node> newNodePtr = std::make_unique<Node>(bestParent, randPoint);
    Node* newNode = newNodePtr.get();
    bestParent->children.emplace_back(std::move(newNodePtr));
    newNode->parent = bestParent;
    newNode->calculateCost();

    nodeManager.addNode(newNode);

    // Rewire nearby nodes
    std::vector<Node*> rewireCandidates;
    nodesInRadiusofPoint(rewireCandidates, rewireRadius, newNode->point);
    for (auto &node : rewireCandidates) {
        if (node->point == newNode->point) continue;
        
        if (node->cost > newNode->cost + newNode->point.distToPoint(node->point)) {
            if (node->parent) {
                auto nodePtr = std::find_if(node->parent->children.begin(), node->parent->children.end(),
                    [node](const std::unique_ptr<Node>& ptr) {
                        return ptr.get() == node;
                });

                newNode->children.push_back(std::move(*nodePtr));
                node->parent->children.erase(nodePtr);
                node->parent = newNode;
                node->calculateCost();
                node->propagateCost();
            }
            
        }
    }
}

Node* Generator::optimalNodeNearGoal() const {
    std::vector<Node*> goalCandidates;
    nodesInRadiusofPoint(goalCandidates, goalRadius, goal);

    if (goalCandidates.size() == 0) {
        return nullptr;
    }

    Node* bestCandidate = goalCandidates[0];

    for (auto& node : goalCandidates) {
        if (node->cost < bestCandidate->cost) {
            bestCandidate = node;
        }
    }

    return bestCandidate;
}