#pragma once 

#include <cmath>
#include <memory>
#include <vector>
#include <random>

#include "point.hpp"
#include "obstacle.hpp"
#include "node.hpp"

namespace RRT {
    struct BoundingBox {
        double minX;
        double minY;
        double maxX;
        double maxY;

        BoundingBox(double minX, double minY, double maxX, double maxY)
            : minX(minX), minY(minY), maxX(maxX), maxY(maxY)
        {}
    };

    class Generator {
        public:
        Point start;
        Point goal;
        BoundingBox bounds;
        std::unique_ptr<Node> root;
        std::vector<Node*> allNodes;
        std::vector<Obstacle> obstacles;

        double stepSize;
        double goalBias;
        double goalRadius;
        double rewireRadius;

        mutable std::mt19937 randomNumberGenerator;

        Generator(Point start, Point goal, BoundingBox bounds, double stepSize, double rewireRadius, double goalBias, double goalRadius, int iterations, const std::vector<Obstacle>& obstacles)
            :   start(start), goal(goal), stepSize(stepSize), rewireRadius(rewireRadius), bounds(bounds), goalBias(goalBias), goalRadius(goalRadius), randomNumberGenerator(std::random_device{}()), obstacles(obstacles)
        {
            allNodes.reserve(iterations);
            root = std::make_unique<Node>(nullptr, start);
            allNodes.push_back(root.get());
        }

        void iterate();
        Node* optimalNodeNearGoal() const;

        private:
        Node* findBestParent(const std::vector<Node*>& nodeList, const Point& point, Node* nearestNode) const;
        void nodesInRadiusofPoint(std::vector<Node*>& nodeList, double radius, const Point& point) const;
        Point genRandPoint() const;
        bool pointIsValid(const Point& p) const;
        Node* nearestNode(const Point& point) const;
    };
};