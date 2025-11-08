#pragma once
#include <cmath>
#include "../third_party/eigen/Dense"

namespace angle_utils {

constexpr double PI = 3.14159265358979323846;

// --- Conversions ---
constexpr double deg2rad(double deg) noexcept {
    return deg * PI / 180.0;
}

constexpr double rad2deg(double rad) noexcept {
    return rad * 180.0 / PI;
}

// --- Angle extraction (for 2D vectors) ---
inline double angle(const Eigen::Vector2d& v) noexcept {
    return std::atan2(v.y(), v.x());  // returns radians
}

// --- Create a vector from angle and magnitude ---
inline Eigen::Vector2d vectorFromAngle(double angle_rad, double magnitude = 1.0) noexcept {
    return Eigen::Vector2d(magnitude * std::cos(angle_rad),
                           magnitude * std::sin(angle_rad));
}

// --- Rotate a vector by an angle (radians) ---
inline Eigen::Vector2d rotate(const Eigen::Vector2d& v, double delta_angle) noexcept {
    Eigen::Rotation2Dd R(delta_angle);
    return R * v;
}

}