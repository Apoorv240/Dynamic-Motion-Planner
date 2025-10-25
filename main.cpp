#include <iostream>
#include <fstream>
#include <memory>
#include <string>
#include <chrono>

#include "math/QuinticSpline.hpp"
#include "planning/Trajectory.hpp"
#include "planning/rrt/rrt.hpp"
#include "planning/spline/spline.hpp"

int main() {
    auto startTime = std::chrono::high_resolution_clock::now();    
    Vec2d start = Vec2d(-170, 0);
    RRT::Generator g(
        start, 
        Vec2d(100,0),//Vec2d(170, 0), 
        RRT::BoundingBox(-182.88, -182.88, 182.88, 182.88),
        10, 20, 0.4, 2, 16000,
        std::vector<RRT::Obstacle> {
            RRT::Obstacle::fromRectBottomLeft(44*2.54, -30*2.54, 10*2.54, 50*2.54),
            RRT::Obstacle::fromRectBottomLeft(-15 * 2.54, -15 * 2.54, 30 * 2.54, 30 * 2.54),
            RRT::Obstacle::fromRectBottomLeft(-51 * 2.54, -27 * 2.54, 10 * 2.54, 50 * 2.54),
        }
    );

    int i = 0;

    int iterations = 4000;
    while (i < iterations) {
        g.iterate();
        i++;
    }

    std::vector<RRT::Node*> path = g.getOptimalPath();
    
    Spline::Generator sg(path, 3);
    sg.parameterize();
    
    sg.calculateKnots();
    sg.calculateControlPoints(1e-3);
    auto points = sg.sampleSpline(100);

    auto endTime = std::chrono::high_resolution_clock::now();

    std::chrono::duration<double, std::milli> duration = endTime - startTime;
    std::cout << "Iterated for " << duration.count() << " milliseconds" << std::endl;
    std::cout << "Iterated " << iterations << " times" << std::endl;
    std::cout << g.nodeManager.size << " Nodes generated" << std::endl;

    std::cout << "t: " << sg.t.size() << std::endl;
    std::cout << "knots: " << sg.knots.size() << std::endl;
    std::cout << "control: " << sg.controlPoints.size() << std::endl << std::endl;

    std::cout << "Generating files..." << std::endl << std::flush;

    std::ofstream outFile("scripts\\outAll.txt");
    for (auto obstacle : g.obstacles) {
        outFile << obstacle.polygon[0].x() << " " << obstacle.polygon[0].y() << std::endl;
        outFile << obstacle.polygon[1].x() << " " << obstacle.polygon[1].y() << std::endl;
        outFile << obstacle.polygon[2].x() << " " << obstacle.polygon[2].y() << std::endl;
        outFile << obstacle.polygon[3].x() << " " << obstacle.polygon[3].y() << std::endl;
    }

    for (const auto& node : g.nodeManager.nodes) {
        auto parent = node->parent;
        if (parent != nullptr) {
            outFile << parent->point.x() << " " << parent->point.y() << " ";
        }
        outFile << node->point.x() << " " << node->point.y() << std::endl; 
    }

    outFile.close();

    std::ofstream outFile2("scripts\\outPath.txt");
    {
        for (auto obstacle : g.obstacles) {
            outFile2 << obstacle.polygon[0].x() << " " << obstacle.polygon[0].y() << std::endl;
            outFile2 << obstacle.polygon[1].x() << " " << obstacle.polygon[1].y() << std::endl;
            outFile2 << obstacle.polygon[2].x() << " " << obstacle.polygon[2].y() << std::endl;
            outFile2 << obstacle.polygon[3].x() << " " << obstacle.polygon[3].y() << std::endl;
        }
        
        //auto node = g.optimalNodeNearGoal();
        std::vector<RRT::Node*> path = g.getOptimalPath();
        // while (node->parent != nullptr) {
        //     auto parent = node->parent;
        //     outFile2 << parent->point.x() << " " << parent->point.y() << " ";
        //     outFile2 << node->point.x() << " " << node->point.y() << std::endl;

        //     node = node->parent;
        // }

        for (auto node : path) {
            if (node->parent) {
                outFile2 << node->parent->point.x() << " " << node->parent->point.y() << " ";
            }
            outFile2 << node->point.x() << " " << node->point.y() << std::endl;
        }
    }   

    outFile2.close();

    std::ofstream outFile3("scripts\\outSpline.txt");
    {
        for (auto obstacle : g.obstacles) {
            outFile3 << obstacle.polygon[0].x() << " " << obstacle.polygon[0].y() << std::endl;
            outFile3 << obstacle.polygon[1].x() << " " << obstacle.polygon[1].y() << std::endl;
            outFile3 << obstacle.polygon[2].x() << " " << obstacle.polygon[2].y() << std::endl;
            outFile3 << obstacle.polygon[3].x() << " " << obstacle.polygon[3].y() << std::endl;
        }

        for (auto point : points) {
            outFile3 << point.x() << " " << point.y() << std::endl;
        }
    }
    outFile3.close();

    // auto goalNode = g.optimalNodeNearGoal();
    // double total = goalNode->cost;
    // std::vector<RRT::Node*> nodes;
    // std::vector<double> t;
    // RRT::Node* node = goalNode;
    // while (node->parent != nullptr) {
    //     t.push_back(node->cost / total);
    //     nodes.push_back(node->parent);
    //     node = node->parent;
    // }
    // nodes.push_back(node);
    // t.push_back(node->cost / total);

    // int p = 3;
    // double n = nodes.size() - 1;
    // std::vector<double> knots(n + p + 2);
    // for (int i = 0; i <= p; i++) {
    //     knots[i] = t[0];
    // }

    // for (int j = 1; j <= n - p; j++) {
    //     double sum = 0;
    //     for (int i = j; i < j + p; i++) {
    //         sum += t[i];
    //     }
    //     knots[j + p] = sum / p;
    // }

    // for (int i = n + 1; i <= n + p + 1; i++) {
    //     knots[i] = t[n];
    // }

    // for (auto i : knots) {
    //     std::cout << i << std::endl;
    // }
    

    // QuinticSegment qp(0, 10, 0, 10, 10, 0);

    // Trajectory t;
    // t.start(Pose2d(0,0)).lineTo(Pose2d(10, 10)).lineTo(Pose2d(20, 20));

    // std::cout << t.head_->_startPose.pos.x() << std::endl;
    // std::cout << t.head_->_endPose.pos.x() << std::endl;
    // std::cout << t.head_->next->_endPose.pos.x() << std::endl;
    // std::cout << t.tail_->_endPose.pos.x() << std::endl;
    // std::cout << t.tail_->previous->_endPose.pos.x() << std::endl;

    // Trajectory t = trajectoryBuilder.start(
    //     x, y, theta
    // ).addConstraints(
    //     maxVel, maxAccel
    // ).splineTo(
    //     x, y, theta
    // ).stop(
    // ).splineTo(
    //     x, y, theta
    // )

    // trajectory.addConstraint(maxVel, maxAccel)

    // std::cout << qp.pos(0) << std::endl;
    // std::cout << qp.pos(1) << std::endl;
    // std::cout << qp.pos(0.5) << std::endl;
}