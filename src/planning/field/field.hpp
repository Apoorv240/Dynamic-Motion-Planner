#pragma once

#include "planning/field/obstacle.hpp"
#include "Eigen/Dense"

namespace field {
    struct Field {
        std::vector<Obstacle> staticObstacles;

        double minX;
        double minY;
        double maxX;
        double maxY;
        
        Field(Eigen::Vector2d bottomLeft, Eigen::Vector2d topRight, const std::vector<field::Obstacle>& obstacles)
            : staticObstacles(obstacles), minX(bottomLeft.x()), minY(bottomLeft.y()), maxX(topRight.x()), maxY(topRight.y())
        {}
    };
}