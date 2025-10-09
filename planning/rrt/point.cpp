#include "point.hpp"

using namespace RRT;

void Point::normalizeDistToPoint(const double distanceToPoint, const Point otherPoint) {
    double dx = x - otherPoint.x;
    double dy = y - otherPoint.y;
    double distance = std::sqrt(dx*dx + dy*dy);
    x = dx / distance * distanceToPoint + otherPoint.x;
    y = dy / distance * distanceToPoint + otherPoint.y;
}