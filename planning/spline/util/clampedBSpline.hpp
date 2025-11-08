#pragma once

#include <vector>

#include "../../../third_party/eigen/Dense"
#include "../../../third_party/eigen/QR"

namespace spline {
    struct Point {
        double t;
        Eigen::Vector2d point;

        Point(double t, Eigen::Vector2d point)
            : t(t), point(point)
        {}
    };

    class ClampedBSpline {
        void parameterize();
        void calculateKnots();
        void calculateControlPoints(double smoothingFactor);
        void calculateDerivativeControlPoints();
        void calculateSecondDerivativeControlPoints();

        double N(int i, double t, unsigned int p) const;

        int degree;
        std::vector<Point> points;
        std::vector<double> knots; // knots
        std::vector<Eigen::Vector2d, Eigen::aligned_allocator<Eigen::Vector2d>> controlPoints;
        std::vector<Eigen::Vector2d, Eigen::aligned_allocator<Eigen::Vector2d>> derivativeControlPoints;
        std::vector<Eigen::Vector2d, Eigen::aligned_allocator<Eigen::Vector2d>> secondDerivativeControlPoints;

    public:
        void generateAll(std::vector<Eigen::Vector2d, Eigen::aligned_allocator<Eigen::Vector2d>> interpPoints, int degree, double smoothingFactor);
        Eigen::Vector2d calculateAt(double t) const;
        Eigen::Vector2d calculateDerivativeAt(double t) const;
        Eigen::Vector2d calculateSecondDerivativeAt(double t) const;
    };
};