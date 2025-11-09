#include "planning/planner.hpp"

void Planner::genGlobalPath() {
    //rrt.iterateIterationsAndUntilFound(RRT_ITERATIONS, RRT_ITERATIONS*3);
    rrt.iterateIterations(RRT_ITERATIONS);
    auto v = rrt.getOptimalPath();
    globalPath.generate(v, 3, 1e-4);
}