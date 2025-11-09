#include "planning/rrt/random.hpp"

Eigen::Vector2d rrt::sampleInBoundsWithBias(const field::Field& field, const Eigen::Vector2d& goal, double goalBias, Random& rng) {
    if (rng.rand() < 0.5) {
        return goal;
    }

    double x = field.minX + rng.rand() * std::abs(field.maxX - field.minX);
    double y = field.minY + rng.rand() * std::abs(field.maxY - field.minY);
    return Eigen::Vector2d(x, y);
}

Eigen::Vector2d rrt::sampleInInformedEllipse(const Eigen::Vector2d& start, const Eigen::Vector2d& goal, double cBest, const field::Field& field, double goalBias, Random& rng) {
    Eigen::Vector2d d = goal - start;
    double cMin = d.norm();
    if (cBest <= cMin) return sampleInBoundsWithBias(field, goal, goalBias, rng);

    double a = cBest * 0.5;
    double b = 0.5 * std::sqrt(cBest*cBest - cMin*cMin);

    Eigen::Vector2d center = (start + goal) * 0.5;

    double angle = std::atan2(d.y(), d.x());
    double ca = std::cos(angle);
    double sa = std::sin(angle);

    double r = std::sqrt(rng.rand());
    double theta = rng.randAngle();

    double xball = r * std::cos(theta);
    double yball = r * std::sin(theta);

    double xs = a * xball;
    double ys = a * yball;

    return Eigen::Vector2d(ca * xs - sa * ys + center.x(), sa * xs + ca * ys + center.y());
}