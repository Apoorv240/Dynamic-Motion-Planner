#pragma once 

#include <cmath>
#include <memory>
#include <vector>
#include <random>

#include "obstacle.hpp"
#include "node.hpp"
#include "nodeManager.hpp"
#include "random.hpp"

#include "../../math/Vec2d.hpp"

namespace RRT {
    class Generator {
        public:
        Vec2d start;
        Vec2d goal;
        BoundingBox bounds;
        std::unique_ptr<Node> root;
        std::vector<Obstacle> obstacles;
        NodeManager nodeManager;

        mutable Random rand;

        double stepSize;
        double goalBias;
        double goalRadius;
        double rewireRadius;

        bool foundPath;
        double bestPathCost;

        Generator(Vec2d start, Vec2d goal, BoundingBox bounds, double stepSize, double rewireRadius, double goalBias, double goalRadius, int iterations, const std::vector<Obstacle>& obstacles)
            :   start(start), 
                goal(goal), 
                stepSize(stepSize), 
                rewireRadius(rewireRadius), 
                bounds(bounds), 
                goalBias(goalBias), 
                goalRadius(goalRadius), 
                rand(), 
                obstacles(obstacles),
                root(std::make_unique<Node>(nullptr, start)),
                foundPath(false),
                bestPathCost(std::numeric_limits<double>::infinity()),
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
        bool lineIsValid(const Vec2d& p1, const Vec2d& p2) const;
        Node* nearestNode(const Vec2d& point) const;
    };
};