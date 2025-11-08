#include "clampedBSpline.hpp"


using namespace spline;

void ClampedBSpline::parameterize() {
    size_t n = points.size();
    if (n == 0) return;

    // 1. Compute cumulative chord lengths
    std::vector<double> chordLengths(n, 0.0);
    for (size_t i = 1; i < n; i++) {
        double dist = (points[i].point - points[i-1].point).norm();
        chordLengths[i] = chordLengths[i-1] + dist;
    }

    double totalLength = chordLengths.back();
    if (totalLength == 0.0) totalLength = 1.0; // avoid divide by zero

    // 2. Normalize cumulative distances to [0,1]
    for (size_t i = 0; i < n; ++i) {
        points[i].t = chordLengths[i] / totalLength;
    }
}

void ClampedBSpline::calculateKnots() {
    double n = points.size() - 1;

    for (int i = 0; i <= degree; i++) {
        knots[i] = 0.0;
    }

    for (int j = 1; j <= n - degree; j++) {
        double sum = 0;

        for (int i = 0; i < degree; i++) {
            sum += points[i + j].t;
        }

        knots[j + degree] = sum / degree;
    }

    for (int i = n + 1; i <= n + degree + 1; i++) {
        knots[i] = 1.0;
    }
}

double ClampedBSpline::N(int i, double t, unsigned int p) const {
    int n = points.size() - 1;
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

void ClampedBSpline::calculateControlPoints(double smoothingFactor) {
    int n = points.size() - 1;

    Eigen::MatrixXd Nmat(n+1, n+1);
    Eigen::VectorXd Px(n+1), Py(n+1);

    for (int i = 0; i <= n; i++) {
        Px(i) = points[i].point.x();
        Py(i) = points[i].point.y();
    }

    for (int i = 0; i <= n; i++) {
        for (int j = 0; j <= n; j++) {
            Nmat(i, j) = N(j, points[i].t, degree);
        }
    }

    Eigen::MatrixXd A = Nmat.transpose() * Nmat;// + smoothingFactor * Eigen::MatrixXd::Identity(n+1, n+1);
    Eigen::VectorXd bx = Nmat.transpose() * Px;
    Eigen::VectorXd by = Nmat.transpose() * Py;

    // normalization factor: average diagonal
    double scale = A.trace() / static_cast<double>(n + 1);
    double lambda = smoothingFactor * scale; // normalized smoothing factor

    Eigen::MatrixXd Reg = A + lambda * Eigen::MatrixXd::Identity(n+1, n+1);

    Eigen::VectorXd ctrlX = Reg.colPivHouseholderQr().solve(bx);
    Eigen::VectorXd ctrlY = Reg.colPivHouseholderQr().solve(by);

    controlPoints.resize(n+1);
    for (int i = 0; i <= n; i++) {
        controlPoints[i] = Eigen::Vector2d(ctrlX(i), ctrlY(i));
    }
}

void ClampedBSpline::calculateDerivativeControlPoints() {
    int n = controlPoints.size() - 1;

    derivativeControlPoints.resize(n);
    for (int i = 0; i < n; i++) {
        double denom = knots[i + degree] - knots[i];
        double coeff = (denom != 0.0) ? (degree / denom) : 0.0;
        derivativeControlPoints[i] = coeff * (controlPoints[i + 1] - controlPoints[i]);
    }
}

void ClampedBSpline::calculateSecondDerivativeControlPoints() {
    int n = derivativeControlPoints.size() - 1;

    secondDerivativeControlPoints.resize(n);
    for (int i = 0; i < n; i++) {
        double denom = knots[i + degree + 1] - knots[i + 2];
        double coeff = (denom != 0.0) ? (degree / denom) : 0.0;
        secondDerivativeControlPoints.push_back(coeff * (derivativeControlPoints[i + 1] - derivativeControlPoints[i]));
    }
}

Eigen::Vector2d ClampedBSpline::calculateAt(double t) const {
    Eigen::Vector2d point(0, 0);

    int n = controlPoints.size() - 1;
    for (int i = 0; i <= n; i++) {
        point += N(i, t, degree) * controlPoints[i];
    }

    return point;
}

Eigen::Vector2d ClampedBSpline::calculateDerivativeAt(double t) const {
    double tStart = knots[degree];
    double tEnd   = knots[knots.size() - degree - 1];
    t = std::clamp(t, tStart + 1e-12, tEnd - 1e-12);

    Eigen::Vector2d point(0, 0);

    for (int i = 0; i < static_cast<int>(derivativeControlPoints.size()); i++) {
        point += N(i + 1, t, degree - 1) * derivativeControlPoints[i];
    }

    return point;
}

Eigen::Vector2d ClampedBSpline::calculateSecondDerivativeAt(double t) const {
    Eigen::Vector2d point(0, 0);

    double tStart = knots[degree];
    double tEnd   = knots[knots.size() - degree - 1];
    t = std::clamp(t, tStart + 1e-12, tEnd - 1e-12);

    for (int i = 0; i < secondDerivativeControlPoints.size(); i++) {
        point += N(i, t, degree - 2) * secondDerivativeControlPoints[i];
    }

    return point;
}

void ClampedBSpline::generateAll(std::vector<Eigen::Vector2d, Eigen::aligned_allocator<Eigen::Vector2d>> interpPoints, int degree, double smoothingFactor) {
    points.reserve(interpPoints.size());
    for (const Eigen::Vector2d& point : interpPoints) {
        points.push_back(Point(0.0, point));
    }

    this->degree = degree;
    this->knots.resize(interpPoints.size() + degree + 1);

    parameterize();
    calculateKnots();
    calculateControlPoints(smoothingFactor);
    calculateDerivativeControlPoints();
    calculateSecondDerivativeControlPoints();
}