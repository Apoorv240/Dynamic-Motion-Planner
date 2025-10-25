#include "spline.hpp"
#include <iostream>

using namespace Spline;

void Generator::parameterize() {
    double totalLength = nodes[nodes.size() - 1]->cost;

    for (const RRT::Node* node : nodes) {
        t.push_back(node->cost/totalLength);
    }
}

void Generator::calculateKnots() {
    double n = nodes.size() - 1;

    for (int i = 0; i <= degree; i++) {
        knots[i] = 0.0;
    }

    for (int j = 1; j <= n - degree; j++) {
        double sum = 0;

        for (int i = 0; i < degree; i++) {
            sum += t[i + j];
        }

        knots[j + degree] = sum / degree;
    }

    for (int i = n + 1; i <= n + degree + 1; i++) {
        knots[i] = 1.0;
    }
}

double Generator::N(int i, double t, unsigned int p) {
    int n = nodes.size() - 1;
    if (p == 0) {
        if (t >= knots[i] && t < knots[i+1]) return 1.0;
        // For the very end (last node), ensure last basis function is active at t==knots.back()
        if (i == n && std::abs(t - knots.back()) < 1e-10) return 1.0;
        return 0.0;
    }

    double leftTerm = 0;
    double rightTerm = 0;

    double leftDenom = knots[i + p] - knots[i];
    if (leftDenom != 0) {
        leftTerm += (t - knots[i]) / leftDenom * N(i, t, p-1);
    }

    double rightDenom = knots[i + p + 1] - knots[i + 1];
    if (rightDenom != 0) {
        rightTerm += (knots[i + p + 1] - t) / rightDenom * N(i+1, t, p-1);
    }

    return leftTerm + rightTerm;
}

void Generator::calculateControlPoints(double smoothingFactor) {
    int n = nodes.size() - 1;

    Eigen::MatrixXd Nmat(n+1, n+1);
    Eigen::VectorXd Px(n+1), Py(n+1);

    for (int i = 0; i <= n; i++) {
        Px(i) = nodes[i]->point.x();
        Py(i) = nodes[i]->point.y();
    }

    for (int i = 0; i <= n; i++) {
        for (int j = 0; j <= n; j++) {
            Nmat(i, j) = N(j, t[i], degree);
        }
    }

    Eigen::MatrixXd A = Nmat.transpose() * Nmat + smoothingFactor * Eigen::MatrixXd::Identity(n+1, n+1);
    Eigen::VectorXd bx = Nmat.transpose() * Px;
    Eigen::VectorXd by = Nmat.transpose() * Py;

    // normalization factor: average diagonal
    double scale = A.trace() / static_cast<double>(n + 1);
    double lambda = smoothingFactor * scale; // normalized smoothing factor

    // Optional safety: clamp alpha to reasonable bounds
    // alpha = std::clamp(alpha, 1e-8, 1.0); // if desired

    Eigen::MatrixXd Reg = A + lambda * Eigen::MatrixXd::Identity(n+1, n+1);

    Eigen::VectorXd ctrlX = Reg.colPivHouseholderQr().solve(bx);
    Eigen::VectorXd ctrlY = Reg.colPivHouseholderQr().solve(by);

    controlPoints.resize(n+1);
    for (int i = 0; i <= n; i++) {
        controlPoints[i] = Eigen::Vector2d(ctrlX(i), ctrlY(i));
    }
}


std::vector<Eigen::Vector2d, Eigen::aligned_allocator<Eigen::Vector2d>> Generator::sampleSpline(int numSamples) {
    std::vector<Eigen::Vector2d, Eigen::aligned_allocator<Eigen::Vector2d>> sampledCurve;
    if (controlPoints.empty()) return sampledCurve;

    int n = controlPoints.size() - 1;
    double tStart = knots[degree];
    double tEnd   = knots[knots.size() - degree - 1];
    double dt = (tEnd - tStart) / (numSamples - 1);

    sampledCurve.reserve(numSamples);

    for (int s = 0; s < numSamples; ++s) {
        double param = tStart + s * dt;
        if (param > tEnd) param = tEnd - 1e-9;
        Eigen::Vector2d pt(0,0);

        for (int i = 0; i <= n; ++i)
            pt += N(i, param, degree) * controlPoints[i];

        sampledCurve.push_back(pt);
    }

    return sampledCurve;
}