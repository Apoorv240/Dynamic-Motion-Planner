#include "planner.hpp"

void Planner::genGlobalPath() {
    rrt.iterateIterations(RRT_ITERATIONS);
    auto v = rrt.getOptimalPath();
    globalPath.generate(v, 3, 1e-4);
}