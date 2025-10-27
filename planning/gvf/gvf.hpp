#pragma once

#include "../spline/spline.hpp"

class GuidingVectorField {
public:
    const Spline& spline;

    GuidingVectorField(const Spline& spline)
        : spline(spline)
    {}

    Eigen::Vector2d calculateVectorAt(Eigen::Vector2d point);
};