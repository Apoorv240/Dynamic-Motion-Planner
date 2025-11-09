#pragma once

#include "planning/rrt/rrt.hpp"
#include "planning/spline/spline.hpp"
#include "planning/gvf/gvf.hpp"
#include "math/Pose2d.hpp"
#include "planning/field/field.hpp"
#include <iostream>

class Planner {
public:
    rrt::Generator rrt;
    spline::FittedSpline globalPath;

    static constexpr double GOAL_BIAS = 0.4;
    static constexpr int RESOLUTION = 20;
    static constexpr double GOAL_RADIUS = 2;
    static constexpr size_t RRT_ITERATIONS = 16000;
    static constexpr double CONVERGENCE_FACTOR = 30;

    Planner(Pose2d startPose, Pose2d goalPose, field::Field field) 
        : rrt(startPose.pos, goalPose.pos, field, RESOLUTION, GOAL_BIAS, GOAL_RADIUS, RRT_ITERATIONS)
    {}

    void genGlobalPath();

    Eigen::Vector2d getDirectionVectorAt(Eigen::Vector2d point) {
        return GuidingVectorField::calculateVectorAt(point, globalPath, CONVERGENCE_FACTOR);
    }
};