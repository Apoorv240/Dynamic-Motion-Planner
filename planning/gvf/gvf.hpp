#pragma once

#include "../spline/spline.hpp"

namespace GuidingVectorField {
    Eigen::Vector2d calculateVectorAt(Eigen::Vector2d point, const Spline& spline, double convergenceFactor);
}