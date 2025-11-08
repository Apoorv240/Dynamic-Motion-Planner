#include "planner.hpp"

void Planner::genGlobalPath() {
    rrt.iterateIterations(RRT_ITERATIONS);
    auto v = rrt.getOptimalPath();
    globalPath = Spline(v, 3);
    globalPath.generate(1e-5);
}