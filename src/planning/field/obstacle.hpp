#pragma once

#include <vector>
#include <Eigen/Dense>

namespace field {
    struct Obstacle {
        std::vector<Eigen::Vector2d, Eigen::aligned_allocator<Eigen::Vector2d>> polygon;

        // Clockwise winding polygon
        Obstacle(const std::vector<Eigen::Vector2d, Eigen::aligned_allocator<Eigen::Vector2d>>& polygon)
            : polygon(polygon)
        {}

        static Obstacle fromRectVertices(double leftX, double bottomY, double rightX, double topY) {
            return Obstacle(
                std::vector<Eigen::Vector2d, Eigen::aligned_allocator<Eigen::Vector2d>>{
                    Eigen::Vector2d(leftX, bottomY),
                    Eigen::Vector2d(rightX, bottomY),
                    Eigen::Vector2d(rightX, topY),
                    Eigen::Vector2d(leftX, topY)
                }
            );
        }

        static Obstacle fromRectBottomLeft(double blX, double blY, double width, double height) {
            return Obstacle(
                std::vector<Eigen::Vector2d, Eigen::aligned_allocator<Eigen::Vector2d>>{
                    Eigen::Vector2d(blX, blY),
                    Eigen::Vector2d(blX + width, blY),
                    Eigen::Vector2d(blX + width, blY + height),
                    Eigen::Vector2d(blX, blY + height)
                }
            );
        }

        static Obstacle fromRectBottomLeftExpanded(double blX, double blY, double width, double height, double expansion) {
            return fromRectBottomLeft(
                blX - expansion,
                blY - expansion,
                width + expansion*2,
                height + expansion*2
            );
        }

        bool pointInObstacle(const Eigen::Vector2d& p) const;
        bool lineInObstacle(const Eigen::Vector2d& p1, const Eigen::Vector2d& p2) const;
    };
}