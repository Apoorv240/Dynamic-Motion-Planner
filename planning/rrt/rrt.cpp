#include "rrt.hpp"
#include <algorithm>

using namespace RRT;

double Node::calculateCost() {
    if (parent == nullptr) {
        return 0;
    }
    double newCost = parent->cost + point.distToPoint(parent->point);
    cost = newCost;
    return newCost;
}

void Node::propagateCost() {
    for (auto &child : children) {
        child->calculateCost();
        child->propagateCost();
    }
}

void Point::normalizeDistToPoint(const double distanceToPoint, const Point otherPoint) {
    double dx = x - otherPoint.x;
    double dy = y - otherPoint.y;
    double distance = std::sqrt(dx*dx + dy*dy);
    x = dx / distance * distanceToPoint + otherPoint.x;
    y = dy / distance * distanceToPoint + otherPoint.y;
}

bool Obstacle::pointInObstacle(const Point& p) const {
    bool inside = false;

    int n = polygon.size();
    for (int i = 0, j = n-1; i < n; j=i++) {
        const Point& polyPointi = polygon[i];
        const Point& polyPointj = polygon[j];

        bool intersect = (
            ((polyPointi.y > p.y) != (polyPointj.y > p.y)) &&
            (p.x < (polyPointj.x - polyPointi.x) * (p.y - polyPointi.y) / (polyPointj.y - polyPointi.y) + polyPointi.x)
        );

        if (intersect) {
            inside = !inside;
        }
    }

    return inside;
}

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
    Node* nearestNode = allNodes[0];
    double lowestDist = nearestNode->point.distToPoint(point);

    for (const auto& node : allNodes) {
        double dist = node->point.distToPoint(point);
        if (dist < lowestDist) {
            nearestNode = node;
            lowestDist = dist;
        }
    }

    return nearestNode;
}

void Generator::nodesInRadiusofPoint(std::vector<Node*>& nodeList, double radius, const Point& point) const {
    for (const auto& node : allNodes) {
        if (point.distToPoint(node->point) <= radius) {
            nodeList.push_back(node);
        }
    }
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
        stepSize * std::sqrt(std::log(allNodes.size()) / allNodes.size()));
    std::vector<Node*> nearbyNodes;
    nodesInRadiusofPoint(nearbyNodes, searchRadius, randPoint);
    Node* bestParent = findBestParent(nearbyNodes, randPoint, nearestNode);

    // Wire new node to the best parent
    std::unique_ptr<Node> newNodePtr = std::make_unique<Node>(bestParent, randPoint);
    Node* newNode = newNodePtr.get();
    bestParent->children.emplace_back(std::move(newNodePtr));
    newNode->parent = bestParent;
    newNode->calculateCost();
    allNodes.push_back(newNode);

    // Rewire nearby nodes
    std::vector<Node*> rewireCandidates;
    nodesInRadiusofPoint(rewireCandidates, rewireRadius, newNode->point);
    for (auto &node : rewireCandidates) {
        if (node->point == newNode->point) continue;
        
        if (node->cost > newNode->cost + newNode->point.distToPoint(node->point)) {
            if (node->parent){
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