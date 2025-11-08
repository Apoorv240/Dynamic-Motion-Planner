#pragma once

#include <vector>
#include "../../../third_party/eigen/Dense"
#include "../../../third_party/eigen/QR"


namespace spline {
    class ArcLengthTable {
        std::vector<double> tSamples;
        std::vector<double> sSamples;

    public:
        double maxS;

        void generate(const std::vector<double>& tSamples, const std::vector<Eigen::Vector2d, Eigen::aligned_allocator<Eigen::Vector2d>>& points);
        double getT(double s) const;
    };
}