#pragma once

#include "rrt/rrt.hpp"
#include "spline/spline.hpp"
#include "gvf/gvf.hpp"
#include "../math/Pose2d.hpp"
#include <iostream>

class Planner {
public:
    RRT::Generator rrt;
    Spline globalPath;

    static constexpr double GOAL_BIAS = 0.4;
    static constexpr int RESOLUTION = 20;
    static constexpr double GOAL_RADIUS = 2;
    static constexpr size_t RRT_ITERATIONS = 2000;
    static constexpr double CONVERGENCE_FACTOR = 10;

    Planner(Pose2d startPose, Pose2d goalPose, RRT::BoundingBox bounds, std::vector<RRT::Obstacle>& obstacles) 
        : rrt(startPose.pos, goalPose.pos, bounds, RESOLUTION, GOAL_BIAS, GOAL_RADIUS, RRT_ITERATIONS, obstacles)
    {}

    void genGlobalPath() {
        std::cout << "Starting iterations" << std::endl;
        rrt.iterateIterations(RRT_ITERATIONS);
        std::cout << "Done iterating" << std::endl;

        auto v = rrt.getOptimalPath();
        globalPath = Spline(v, 3);
        
        std::cout << "Generating spline" << std::endl;
        globalPath.generate(1e-5);
    }

    Eigen::Vector2d getDirectionVectorAt(Eigen::Vector2d point) {
        return GuidingVectorField::calculateVectorAt(point, globalPath, CONVERGENCE_FACTOR);
    }
};