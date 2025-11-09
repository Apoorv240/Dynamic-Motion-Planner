#pragma once

#include <random>
#include <cmath>
#include <Eigen/Dense>
#include "planning/field/field.hpp"

namespace rrt {
    constexpr double PI = 3.14159265358979323846;

    struct Random {
        std::mt19937_64 gen;
        std::uniform_real_distribution<double> unif{0.0, 1.0};
        std::uniform_real_distribution<double> unifAngle{0.0, 2.0*PI}; // radians

        Random(uint64_t seed = std::random_device{}()) : gen(seed) {}

        double rand() { return unif(gen); }
        double randAngle() { return unifAngle(gen); }
    };

    Eigen::Vector2d sampleInBoundsWithBias(const field::Field& field, const Eigen::Vector2d& goal, double goalBias, Random& rng);

    Eigen::Vector2d sampleInInformedEllipse(const Eigen::Vector2d& start, const Eigen::Vector2d& goal, double cBest, const field::Field& field, double goalBias, Random& rng);
}