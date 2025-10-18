#include "obstacle.hpp"

using namespace RRT;

bool Obstacle::pointInObstacle(const Vec2d& p) const {
    bool inside = false;

    int n = polygon.size();
    for (int i = 0, j = n-1; i < n; j=i++) {
        const Vec2d& polyPointi = polygon[i];
        const Vec2d& polyPointj = polygon[j];

        bool intersect = (
            ((polyPointi.y() > p.y()) != (polyPointj.y() > p.y())) &&
            (p.x() < (polyPointj.x() - polyPointi.x()) * (p.y() - polyPointi.y()) / (polyPointj.y() - polyPointi.y()) + polyPointi.x())
        );

        if (intersect) {
            inside = !inside;
        }
    }

    return inside;
}