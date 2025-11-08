#pragma once

#include <vector>
#include <algorithm>

#include "../rrt/rrt.hpp"
#include "../../third_party/eigen/Dense"
#include "../../third_party/eigen/QR"
#include "util/clampedBSpline.hpp"
#include "util/arcLengthTable.hpp"

namespace spline {
    class FittedSpline {
        std::vector<Eigen::Vector2d, Eigen::aligned_allocator<Eigen::Vector2d>> sampleSplineT(int numSamples) const;
        ArcLengthTable arcLengthTable;

        ClampedBSpline spline;

        void generateArcLengthMapping(int numSamples);

        int degree;
    public:
        void generate(const std::vector<RRT::Node*>& nodes, const int degree, const double smoothingFactor) {
            this->degree = degree;

            std::vector<Eigen::Vector2d, Eigen::aligned_allocator<Eigen::Vector2d>> points;
            points.resize(nodes.size());

            for (size_t i = 0; i < nodes.size(); i++) {
                points[i] = Eigen::Vector2d(nodes[i]->point.x(), nodes[i]->point.y());
            }

            spline.generateAll(points, degree, smoothingFactor);
            generateArcLengthMapping(100);
        }

        Eigen::Vector2d calculateAt(double s) const { return spline.calculateAt(arcLengthTable.getT(s)); }
        Eigen::Vector2d calculateDerivativeAt(double s) const { return spline.calculateDerivativeAt(arcLengthTable.getT(s)); }
        Eigen::Vector2d calculateSecondDerivativeAt(double s) const { return spline.calculateSecondDerivativeAt(arcLengthTable.getT(s)); }
        
        double nearestS(Eigen::Vector2d point, int numSamples) const;
        std::vector<Eigen::Vector2d, Eigen::aligned_allocator<Eigen::Vector2d>> sampleSpline(int numSamples) const;
    };
}