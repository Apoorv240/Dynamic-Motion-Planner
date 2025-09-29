#pragma once

class Point2d {
private:
    double x_;
    double y_;

public:
    Point2d(double x, double y)
        : x_(x), y_(y)
    {}

    Point2d() = default;

    double x() { return x_; }
    double y() { return y_; } 
};