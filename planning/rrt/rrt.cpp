#include "rrt.hpp"
#include <algorithm>

using namespace RRT;

Node* Node::getParentRaw() const {
    if (auto sp = parent.lock()) {
        return sp.get();
    }
    return nullptr;
}

double Node::calculateCost() {
    Node* sparent = getParentRaw();
    if (sparent == nullptr) {
        return 0;
    }
    double newCost = sparent->cost + point.distToPoint(sparent->point);
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

Point Generator::genRandPoint() const {
    std::mt19937 gen(std::random_device{}());
    std::uniform_real_distribution<> bias(0, 1);
    if (bias(gen) < goalBias) {
        return goal;
    }
    std::uniform_real_distribution<> distX(bounds.minX, bounds.maxX);
    std::uniform_real_distribution<> distY(bounds.minY, bounds.maxY);
    return Point(distX(gen), distY(gen));
}

std::shared_ptr<Node> Generator::nearestNode(const Point& point) const {
    std::shared_ptr<Node> nearestNode = allNodes[0];
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

void Generator::nodesInRadiusofPoint(std::vector<std::shared_ptr<Node>>& nodeList, double radius, const Point point) const {
    for (const auto& node : allNodes) {
        if (point.distToPoint(node->point) <= radius) {
            nodeList.push_back(node);
        }
    }
}

std::shared_ptr<Node> Generator::findBestParent(const std::vector<std::shared_ptr<Node>>& nodeList, const Point& point, const std::shared_ptr<Node> nearestNode) const {
    std::shared_ptr<Node> bestParent = nearestNode;
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

void Generator::iterate() {
    Point randPoint = genRandPoint();

    // Nearest Node to Point
    std::shared_ptr<Node> nearestNode = this->nearestNode(randPoint);

    // Normalize the point to the node
    if (randPoint == nearestNode->point) {
        return;
    }
    randPoint.normalizeDistToPoint(stepSize, nearestNode->point);

    // TODO: CHECK COLLISION

    // Choose best parent
    double searchRadius = std::max(stepSize * 2, 
        stepSize * std::sqrt(std::log(allNodes.size()) / allNodes.size()));
    std::vector<std::shared_ptr<Node>> nearbyNodes;
    nodesInRadiusofPoint(nearbyNodes, searchRadius, randPoint);
    std::shared_ptr<Node> bestParent = findBestParent(nearbyNodes, randPoint, nearestNode);

    // Wire new node to the best parent
    auto newNode = std::make_shared<Node>(bestParent, randPoint);
    bestParent->children.push_back(newNode);
    newNode->parent = bestParent;
    newNode->calculateCost();
    allNodes.push_back(newNode);

    // Rewire nearby nodes
    std::vector<std::shared_ptr<Node>> rewireCandidates;
    nodesInRadiusofPoint(rewireCandidates, stepSize*3, newNode->point);
    for (auto &node : rewireCandidates) {
        if (node == newNode) continue;
        if (node->cost > newNode->cost + newNode->point.distToPoint(node->point)) {
            auto oldParent = node->parent.lock();
            if (oldParent){
                node->parent = newNode;
                newNode->children.push_back(node);
                
                auto& siblings = oldParent->children;
                siblings.erase(std::remove(siblings.begin(), siblings.end(), node), siblings.end());
                node->calculateCost();
                node->propagateCost();
            }
            
        }
    }
}