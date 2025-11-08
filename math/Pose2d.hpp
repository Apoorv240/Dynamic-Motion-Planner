#pragma once

#include "Vec2d.hpp"

struct Pose2d {
    Vec2d pos;
    double heading;
    Vec2d vel;
    Vec2d acc;

    Pose2d(double x, double y) 
        : pos(x, y), vel(0, 0), acc(0, 0)
    {}

    Pose2d(Vec2d pos, Vec2d vel, Vec2d acc) 
        : pos(pos), vel(vel), acc(acc)
    {}

    Pose2d() = default;

    Pose2d operator+(const Pose2d& other) const {
        return Pose2d(pos + other.pos, vel + other.vel, acc + other.acc);
    }
};