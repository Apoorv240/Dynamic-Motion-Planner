#pragma once 

#include <cmath>
#include <memory>
#include <vector>
#include <random>

namespace RRT {
    struct Point {
        double x;
        double y;

        Point(double x, double y) 
            : x(x), y(y) 
        {}

        inline double distToPoint(const Point& p) const {
            return std::sqrt((p.x - this->x) * (p.x - this->x) + (p.y - this->y) * (p.y - this->y));
        }

        bool operator==(const Point &p) const {
            return (
                std::abs(x - p.x) < 1e-9 &&
                std::abs(y - p.y) < 1e-9
            );
        }
        
        void normalizeDistToPoint(const double length, const Point otherPoint);
    };

    struct Obstacle {
        std::vector<Point> polygon;

        // Clockwise winding polygon
        Obstacle(const std::vector<Point>& polygon)
            : polygon(polygon)
        {}

        bool pointInObstacle(const Point& p) const;
    };

    struct Node {
        std::weak_ptr<Node> parent;
        std::vector<std::shared_ptr<Node>> children;
        Point point;
        double cost;

        Node(std::shared_ptr<Node> parent, Point point)
            : parent(parent), children(), point(point), cost(0)
        {}

        Node* getParentRaw() const;

        double calculateCost();
        void propagateCost();
    };

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
        std::shared_ptr<Node> root;
        std::vector<std::shared_ptr<Node>> allNodes;
        std::vector<Obstacle> obstacles;

        double stepSize;
        double goalBias;

        mutable std::mt19937 randomNumberGenerator;

        Generator(Point start, Point goal, BoundingBox bounds, double stepSize, double goalBias, int iterations, const std::vector<Obstacle>& obstacles)
            :   start(start), goal(goal), stepSize(stepSize), bounds(bounds), goalBias(goalBias), randomNumberGenerator(std::random_device{}()), obstacles(obstacles)
        {
            allNodes.reserve(iterations);
            root = std::make_shared<Node>(nullptr, start);
            allNodes.push_back(root);
        }

        void iterate();

        private:
        std::shared_ptr<Node> findBestParent(const std::vector<std::shared_ptr<Node>>& nodeList, const Point& point, const std::shared_ptr<Node> nearestNode) const;
        void nodesInRadiusofPoint(std::vector<std::shared_ptr<Node>>& nodeList, double radius, const Point point) const;
        Point genRandPoint() const;
        bool pointIsValid(const Point& p) const;
        std::shared_ptr<Node> nearestNode(const Point& point) const;
    };
};