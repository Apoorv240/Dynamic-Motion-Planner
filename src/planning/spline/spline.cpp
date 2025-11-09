#include "planning/spline/spline.hpp"
#include <iostream>

using namespace spline;


std::vector<Eigen::Vector2d, Eigen::aligned_allocator<Eigen::Vector2d>> FittedSpline::sampleSplineT(int numSamples) const {
    std::vector<Eigen::Vector2d, Eigen::aligned_allocator<Eigen::Vector2d>> sampledCurve;

    double tStart = 0;
    double tEnd = 1;
    double dt = (tEnd - tStart) / (numSamples - 1);

    sampledCurve.reserve(numSamples);

    for (int s = 0; s < numSamples; s++) {
        double param = tStart + s * dt;
        if (param > tEnd) param = tEnd - 1e-9;

        sampledCurve.push_back(spline.calculateAt(param));
    }

    return sampledCurve;
}

std::vector<Eigen::Vector2d, Eigen::aligned_allocator<Eigen::Vector2d>> FittedSpline::sampleSpline(int numSamples) const {
    std::vector<Eigen::Vector2d, Eigen::aligned_allocator<Eigen::Vector2d>> sampledCurve;

    double tStart = 0;
    double tEnd = 1;

    double sStart = 0;
    double sEnd = arcLengthTable.maxS;
    double dS = (sEnd - sStart) / (numSamples - 1);

    sampledCurve.reserve(numSamples);

    for (int i = 0; i < numSamples; i++) {
        double param = sStart + i * dS;
        if (param > sEnd) param = sEnd - 1e-9;

        sampledCurve.push_back(calculateAt(param));
    }

    return sampledCurve;
}


double FittedSpline::nearestS(Eigen::Vector2d point, int numSamples) const {
    // Coarsely sample curve
    auto sampledCurve = sampleSpline(numSamples);

    double sStart = 0;
    double sEnd = arcLengthTable.maxS;
    double dS = (sEnd - sStart) / (numSamples - 1);

    double bestS = 0;
    double bestSquaredDist = 1e9;
    {
        double s = 0;
        for (const Eigen::Vector2d &vec : sampledCurve) {
            double dist = (point - vec).squaredNorm();
            if (dist < bestSquaredDist) {
                bestS = s;
                bestSquaredDist = dist;
            }
            s += dS;
        }
    }

    // Newton-Rhapson
    double s = bestS;
    double epsilon = 1e-6;
    int maxIter = 15;

    while (maxIter-- > 0) {
        Eigen::Vector2d Ct = calculateAt(s);
        Eigen::Vector2d Cdt = calculateDerivativeAt(s);
        Eigen::Vector2d Cddt = calculateSecondDerivativeAt(s);
        double distanceDerivative      = 2 * (Ct - point).dot(Cdt);
        double distanceSecondDerivative = 2 * (Cdt.dot(Cdt) + (Ct - point).dot(Cddt));

        if (std::abs(distanceSecondDerivative) < 1e-8 || std::isnan(distanceSecondDerivative)) {
            break; // Can't safely step
        }
        double sNext = s - distanceDerivative / distanceSecondDerivative;

        // Clamp t to curve parameter range
        s = std::clamp(sNext, sStart, sEnd);

        // Optional: fast exit if already close
        if (std::abs(distanceDerivative) < epsilon) {
            break;
        }
    }

    return bestS;
}

void FittedSpline::generateArcLengthMapping(int numSamples) {
    double dt = 1.0 / (numSamples - 1);

    std::vector<double> tSamples;

    auto points = sampleSplineT(numSamples);
    tSamples.resize(points.size());

    tSamples[0] = 0;
    for (size_t i = 1; i < points.size(); i++) {
        tSamples[i] = tSamples[i-1] + dt;
    }

    arcLengthTable.generate(tSamples, points);
}