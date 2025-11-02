#include "gvf.hpp"
#include <iostream>

Eigen::Vector2d GuidingVectorField::calculateVectorAt(Eigen::Vector2d point) {
    double t = spline.nearestS(point, 1000);
    //Eigen::Vector2d tangent = spline.calculateAt(t + 0.01) - spline.calculateAt(t);
    Eigen::Vector2d tangent = spline.calculateDerivativeAt(t);
    Eigen::Vector2d perpendicular = (spline.calculateAt(t) - point) * convergenceFactor;
    return (tangent + perpendicular).normalized();
}