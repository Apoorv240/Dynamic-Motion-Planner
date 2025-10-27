#pragma once

#include "../spline/spline.hpp"

class GuidingVectorField {
public:
    const Spline& spline;
    double convergenceFactor;

    GuidingVectorField(const Spline& spline, double convergenceFactor)
        : spline(spline), convergenceFactor(convergenceFactor)
    {}

    Eigen::Vector2d calculateVectorAt(Eigen::Vector2d point);
};