#include "obstacle.hpp"

using namespace RRT;

bool Obstacle::pointInObstacle(const Point& p) const {
    bool inside = false;

    int n = polygon.size();
    for (int i = 0, j = n-1; i < n; j=i++) {
        const Point& polyPointi = polygon[i];
        const Point& polyPointj = polygon[j];

        bool intersect = (
            ((polyPointi.y > p.y) != (polyPointj.y > p.y)) &&
            (p.x < (polyPointj.x - polyPointi.x) * (p.y - polyPointi.y) / (polyPointj.y - polyPointi.y) + polyPointi.x)
        );

        if (intersect) {
            inside = !inside;
        }
    }

    return inside;
}