#pragma once

#include "../math/Pose2d.hpp"
#include "../planning/gvf/gvf.hpp"
#include "controller/pid.hpp"
#include <chrono>
#include <iostream>

class Robot {
public:
    Pose2d pose;

    double maxVel;

    PIDController xController;
    PIDController yController;

    Robot(Pose2d startPose)
        : pose(startPose), xController(0.1, 0, 0), yController(0.1, 0, 0), maxVel(0.05) // 5 cm/s
    {}

    void update(Vec2d targetPos, long long pauseTime) {
        auto ms = std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::system_clock::now().time_since_epoch()).count();
        pose.vel.setX(xController.update(pose.pos.x(), targetPos.x(), ms));
        pose.vel.setY(yController.update(pose.pos.y(), targetPos.y(), ms));
        pose.vel = pose.vel.normalized() * maxVel;
        //std::cout << pose.vel.x() << std::endl << std::flush;
        std::this_thread::sleep_for(std::chrono::milliseconds(pauseTime));
        auto dt = std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::system_clock::now().time_since_epoch()).count() - ms;
        pose.pos.setX(pose.pos.x() + pose.vel.x() * dt);
        pose.pos.setY(pose.pos.y() + pose.vel.y() * dt);
    }

    void update(Eigen::Vector2d directionVector, double pauseTime) {
        update(pose.pos + Vec2d(directionVector.x(), directionVector.y()), pauseTime);
    }

    bool atTarget(Vec2d target, double radius) {
        return std::abs((target - pose.pos).magnitude()) < radius;
    }
};