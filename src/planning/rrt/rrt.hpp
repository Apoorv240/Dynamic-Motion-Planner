#pragma once 

#include <cmath>
#include <memory>
#include <vector>
#include <random>

#include "planning/rrt/node.hpp"
#include "planning/rrt/nodeManager.hpp"
#include "planning/rrt/random.hpp"

#include <Eigen/Dense>
#include "planning/field/field.hpp"

namespace rrt {
    class Generator {
    public:
        Generator(Eigen::Vector2d start, Eigen::Vector2d goal, field::Field field, double stepSize, double goalBias, double goalRadius, int iterations)
            :   start(start), 
                goal(goal), 
                field(field),
                stepSize(stepSize), 
                goalBias(goalBias), 
                goalRadius(goalRadius), 
                rand(), 
                root(std::make_unique<Node>(nullptr, start)),
                foundPath(false),
                bestPathCost(std::numeric_limits<double>::infinity()),
                bestPathDistance(std::numeric_limits<double>::infinity()),
                nodeManager(iterations)
        {
            root = std::make_unique<Node>(nullptr, start);
            nodeManager.addNode(root.get());
        }

        void iterate();
        int iterateUntilPathFound(int maxIter);
        void iterateIterations(int iter);
        int iterateIterationsAndUntilFound(int minIter, int maxIter);
        Node* optimalNodeNearGoal() const;
        std::vector<Node*> getOptimalPath() const;
        field::Field field;
        NodeManager nodeManager;

    private:
        Eigen::Vector2d start;
        Eigen::Vector2d goal;
        std::unique_ptr<Node> root;

        mutable Random rand;

        double stepSize;
        double goalBias;
        double goalRadius;

        bool foundPath;
        double bestPathCost;
        double bestPathDistance;

        Node* findBestParent(const std::vector<Node*>& nodeList, const Eigen::Vector2d& point, Node* nearestNode) const;
        void nodesInRadiusofPoint(std::vector<Node*>& nodeList, double radius, const Eigen::Vector2d& point) const;
        Eigen::Vector2d genRandPoint() const;
        bool pointIsValid(const Eigen::Vector2d& p) const;
        bool lineIsValid(const Eigen::Vector2d& p1, const Eigen::Vector2d& p2) const;
        Node* nearestNode(const Eigen::Vector2d& point) const;
    };
};