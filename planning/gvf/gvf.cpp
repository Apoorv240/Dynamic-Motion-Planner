#include "gvf.hpp"
#include <iostream>

Eigen::Vector2d GuidingVectorField::calculateVectorAt(Eigen::Vector2d point) {
    double t = spline.nearestT(point, 100);
    Eigen::Vector2d tangent = spline.calculateDerivativeAt(t);
    Eigen::Vector2d perpendicular = (spline.calculateAt(t) - point) * convergenceFactor;
    return (perpendicular + tangent).normalized() * perpendicular.norm();
}