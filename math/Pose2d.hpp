#pragma once

#include "../third_party/eigen/Dense"
#include "../third_party/eigen/QR"

#include "angle.hpp"

struct Pose2d {
    Eigen::Vector2d pos;
    Eigen::Vector2d heading;
    Eigen::Vector2d vel;
    Eigen::Vector2d acc;

    Pose2d(double x, double y, double heading) 
        : pos(x, y), heading(angle_utils::vectorFromAngle(heading)), vel(0, 0), acc(0, 0)
    {}

    Pose2d(Eigen::Vector2d pos, Eigen::Vector2d heading, Eigen::Vector2d vel, Eigen::Vector2d acc) 
        : pos(pos), vel(vel), heading(heading), acc(acc)
    {}

    Pose2d() = default;

    Pose2d operator+(const Pose2d& other) const {
        return Pose2d(pos + other.pos, heading + other.heading, vel + other.vel, acc + other.acc);
    }
};