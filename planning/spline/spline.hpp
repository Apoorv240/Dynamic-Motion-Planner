#pragma once

#include <vector>

#include "../rrt/rrt.hpp"
#include "../../third_party/eigen/Dense"
#include "../../third_party/eigen/QR"

namespace Spline {

    class Generator {
    public:
        std::vector<RRT::Node*>& nodes;
        unsigned int degree;
        Generator(std::vector<RRT::Node*>& nodes, unsigned int degree)
            :  nodes(nodes), degree(degree), knots(nodes.size() + degree + 1)
        {
        }

        void parameterize();
        void calculateKnots();
        void calculateControlPoints();
        std::vector<Eigen::Vector2d, Eigen::aligned_allocator<Eigen::Vector2d>> sampleSpline(int numSamples);
    
        std::vector<double> t; // t
        std::vector<double> knots; // knots
        std::vector<Eigen::Vector2d, Eigen::aligned_allocator<Eigen::Vector2d>> controlPoints;
        double N(int i, double t, unsigned int p);
    };
};
