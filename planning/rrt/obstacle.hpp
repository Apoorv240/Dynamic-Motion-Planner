#pragma once

#include <vector>

#include "point.hpp"

namespace RRT {
    struct Obstacle {
        std::vector<Point> polygon;

        // Clockwise winding polygon
        Obstacle(const std::vector<Point>& polygon)
            : polygon(polygon)
        {}

        static Obstacle fromRectVertices(double leftX, double bottomY, double rightX, double topY) {
            return Obstacle(
                std::vector<Point>{
                    Point(leftX, bottomY),
                    Point(rightX, bottomY),
                    Point(rightX, topY),
                    Point(leftX, topY)
                }
            );
        }

        static Obstacle fromRectBottomLeft(double blX, double blY, double width, double height) {
            return Obstacle(
                std::vector<Point>{
                    Point(blX, blY),
                    Point(blX + width, blY),
                    Point(blX + width, blY + height),
                    Point(blX, blY + height)
                }
            );
        }

        bool pointInObstacle(const Point& p) const;
    };
}