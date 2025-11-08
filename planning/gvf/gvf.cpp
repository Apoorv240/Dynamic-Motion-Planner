#include "gvf.hpp"
#include <iostream>

Eigen::Vector2d GuidingVectorField::calculateVectorAt(Eigen::Vector2d point, const Spline& spline, double convergenceFactor) {
    double t = spline.nearestS(point + Eigen::Vector2d(1e-9, 1e-9), 1000);
    Eigen::Vector2d tangent = spline.calculateDerivativeAt(t) + Eigen::Vector2d(1e-9, 1e-9);
    Eigen::Vector2d secondDerivative = spline.calculateSecondDerivativeAt(t) + Eigen::Vector2d(1e-9, 1e-9);
    double curvature = 0;//std::abs(tangent.cross(secondDerivative)) / std::pow(tangent.norm(), 3);
    Eigen::Vector2d perpendicular = (spline.calculateAt(t) - point) * convergenceFactor;
    return (tangent + perpendicular).normalized() * perpendicular.norm() / (1+curvature);
}