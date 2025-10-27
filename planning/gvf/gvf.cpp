#include "gvf.hpp"
#include <iostream>

Eigen::Vector2d GuidingVectorField::calculateVectorAt(Eigen::Vector2d point) {
    double t = spline.nearestT(point, 100);
    std::cout << t << std::endl;
    Eigen::Vector2d tangent = spline.calculateDerivativeAt(t);
    Eigen::Vector2d perpendicular = 10*(spline.calculateAt(t) - point);
    return (perpendicular + tangent).normalized() * 5;
}