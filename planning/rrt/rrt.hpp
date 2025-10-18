#pragma once 

#include <cmath>
#include <memory>
#include <vector>
#include <random>

#include "point.hpp"
#include "obstacle.hpp"
#include "node.hpp"
#include "nodeManager.hpp"

#include "../../math/Vec2d.hpp"

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
        Vec2d start;
        Vec2d goal;
        BoundingBox bounds;
        std::unique_ptr<Node> root;
        //std::vector<Node*> allNodes;
        std::vector<Obstacle> obstacles;
        NodeManager nodeManager;

        double stepSize;
        double goalBias;
        double goalRadius;
        double rewireRadius;

        mutable std::mt19937 randomNumberGenerator;

        Generator(Vec2d start, Vec2d goal, BoundingBox bounds, double stepSize, double rewireRadius, double goalBias, double goalRadius, int iterations, const std::vector<Obstacle>& obstacles)
            :   start(start), 
                goal(goal), 
                stepSize(stepSize), 
                rewireRadius(rewireRadius), 
                bounds(bounds), 
                goalBias(goalBias), 
                goalRadius(goalRadius), 
                randomNumberGenerator(std::random_device{}()), 
                obstacles(obstacles),
                root(std::make_unique<Node>(nullptr, start)),
                nodeManager(iterations)
        {
            //allNodes.reserve(iterations);
            root = std::make_unique<Node>(nullptr, start);

            //allNodes.push_back(root.get());
            nodeManager.addNode(root.get());
        }

        void iterate();
        Node* optimalNodeNearGoal() const;

        private:
        Node* findBestParent(const std::vector<Node*>& nodeList, const Vec2d& point, Node* nearestNode) const;
        void nodesInRadiusofPoint(std::vector<Node*>& nodeList, double radius, const Vec2d& point) const;
        Vec2d genRandPoint() const;
        bool pointIsValid(const Vec2d& p) const;
        Node* nearestNode(const Vec2d& point) const;
    };
};