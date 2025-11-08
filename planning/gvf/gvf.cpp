#include "gvf.hpp"
#include <iostream>

Eigen::Vector2d GuidingVectorField::calculateVectorAt(Eigen::Vector2d point, const spline::FittedSpline& spline, double convergenceFactor) {
    double s = spline.nearestS(point, 1000);
    Eigen::Vector2d tangent = spline.calculateDerivativeAt(s);
    Eigen::Vector2d perpendicular = (spline.calculateAt(s) - point);

    if (perpendicular.norm() < 1e-1) {
        return tangent.normalized();
    }

    return (tangent + perpendicular * convergenceFactor).normalized();
}