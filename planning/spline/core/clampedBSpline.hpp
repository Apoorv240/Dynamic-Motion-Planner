#pragma once

#include <vector>

namespace spline {
    struct Point {
        double t;
        double point;

        Point(double t, double point)
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
        std::vector<double> controlPoints;
        std::vector<double> derivativeControlPoints;
        std::vector<double> secondDerivativeControlPoints;
        
    public:
        ClampedBSpline(int degree, std::vector<double> interpPoints)
            : degree(degree)
        {
            points.reserve(interpPoints.size());
            for (const double& point : interpPoints) {
                points.push_back(Point(0.0, point));
            }
        }

        void generateAll(double smoothingFactor);
        double calculateAt(double t) const;
        double calculateDerivativeAt(double t) const;
        double calculateSecondDerivativeAt(double t) const;
    };
};