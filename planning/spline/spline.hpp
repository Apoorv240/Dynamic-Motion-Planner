#pragma once

#include <vector>

#include "../rrt/rrt.hpp"
#include "../../third_party/eigen/Dense"
#include "../../third_party/eigen/QR"

class Spline {
public:
    std::vector<RRT::Node*>& nodes;
    unsigned int degree;
    Spline(std::vector<RRT::Node*>& nodes, unsigned int degree)
        :  nodes(nodes), degree(degree), knots(nodes.size() + degree + 1)
    {
    }

    void parameterize();
    void calculateKnots();
    void calculateControlPoints(double smoothingFactor);
    void calculateDerivativeControlPoints();
    void calculateSecondDerivativeControlPoints();

    Eigen::Vector2d calculateAt(double t) const;
    Eigen::Vector2d calculateDerivativeAt(double t) const;
    Eigen::Vector2d calculateSecondDerivativeAt(double t) const;
    std::vector<Eigen::Vector2d, Eigen::aligned_allocator<Eigen::Vector2d>> sampleSpline(int numSamples) const;

    std::vector<double> t; // t
    std::vector<double> knots; // knots
    std::vector<Eigen::Vector2d, Eigen::aligned_allocator<Eigen::Vector2d>> controlPoints;
    std::vector<Eigen::Vector2d, Eigen::aligned_allocator<Eigen::Vector2d>> derivativeControlPoints;
    std::vector<Eigen::Vector2d, Eigen::aligned_allocator<Eigen::Vector2d>> secondDerivativeControlPoints;
    double N(int i, double t, unsigned int p) const;
    double nearestT(Eigen::Vector2d point, double numSamples) const;
};
