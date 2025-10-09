#pragma once

#include <cmath>

namespace RRT {
        struct Point {
        double x;
        double y;

        Point(double x, double y) 
            : x(x), y(y) 
        {}

        inline double distToPoint(const Point& p) const {
            return std::sqrt((p.x - this->x) * (p.x - this->x) + (p.y - this->y) * (p.y - this->y));
        }

        bool operator==(const Point &p) const {
            return (
                std::abs(x - p.x) < 1e-9 &&
                std::abs(y - p.y) < 1e-9
            );
        }
        
        void normalizeDistToPoint(const double length, const Point otherPoint);
    };
}