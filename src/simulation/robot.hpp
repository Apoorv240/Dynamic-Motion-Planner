#pragma once

#include "math/Pose2d.hpp"
#include "planning/gvf/gvf.hpp"
#include "simulation/controller/pid.hpp"
#include <chrono>
#include <thread>
#include <iostream>

class Robot {
public:
    Pose2d pose;

    double maxVel;

    PIDController xController;
    PIDController yController;
    PIDController headingController;

    Robot(Pose2d startPose)
        : pose(startPose), xController(0.1, 0, 0), yController(0.1, 0, 0), headingController(0.1, 0, 0), maxVel(0.1) // 5 cm/s
    {}

    void update(Eigen::Vector2d targetPos, Eigen::Vector2d targetHeading, long long pauseTime) {
        auto ms = std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::system_clock::now().time_since_epoch()).count();
        pose.vel.x() = xController.update(pose.pos.x(), targetPos.x(), ms);
        pose.vel.y() = yController.update(pose.pos.y(), targetPos.y(), ms);
        pose.vel = pose.vel.normalized() * maxVel;
        //std::cout << pose.vel.x() << std::endl << std::flush;
        std::this_thread::sleep_for(std::chrono::milliseconds(pauseTime));
        auto dt = std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::system_clock::now().time_since_epoch()).count() - ms;
        pose.pos.x() = pose.pos.x() + pose.vel.x() * dt;
        pose.pos.y() = pose.pos.y() + pose.vel.y() * dt;

        pose.heading.x() = std::cos(headingController.update(std::atan2(pose.heading.y(), pose.heading.x()), std::atan2(targetHeading.y(), targetHeading.x()), ms));
        pose.heading.y() = std::sin(headingController.update(std::atan2(pose.heading.y(), pose.heading.x()), std::atan2(targetHeading.y(), targetHeading.x()), ms));
    }

    void update(Eigen::Vector2d directionVector, double pauseTime) {
        update(pose.pos + directionVector, directionVector.normalized(), pauseTime);
    }

    bool atTarget(Eigen::Vector2d target, double radius) {
        return std::abs((target - pose.pos).norm()) < radius;
    }
};