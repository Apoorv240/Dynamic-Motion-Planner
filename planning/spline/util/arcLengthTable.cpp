#include "arcLengthTable.hpp"

using namespace spline;

void ArcLengthTable::generate(const std::vector<double>& tSamples, const std::vector<Eigen::Vector2d, Eigen::aligned_allocator<Eigen::Vector2d>>& points) {
    this->tSamples = tSamples;
    sSamples.resize(points.size());
    sSamples[0] = 0.0;
    for (size_t i = 1; i < points.size(); i++) {
        auto dS = (points[i] - points[i-1]).norm();
        sSamples[i] = sSamples[i-1] + dS;
    }
    maxS = sSamples.back();
}

double ArcLengthTable::getT(double s) const {
    if (s < 0) return 0;
    if (s > sSamples.back()) return sSamples.back();
    auto lower = std::lower_bound(sSamples.begin(), sSamples.end(), s);
    size_t i = std::distance(sSamples.begin(), lower) - 1;

    double t0 = tSamples[i];
    double t1 = tSamples[i+1];
    double s0 = sSamples[i];
    double s1 = sSamples[i+1];

    double t = t0 + (s - s0) / (s1 - s0) * (t1 - t0);

    return t;
}