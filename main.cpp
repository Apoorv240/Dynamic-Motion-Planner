#include <iostream>
#include <fstream>
#include <memory>
#include <string>

#include "math/QuinticSpline.hpp"
#include "planning/Trajectory.hpp"
#include "planning/rrt/rrt.hpp"

int main() {
    RRT::Generator g(
        RRT::Point(-170, 0), 
        RRT::Point(170, 0), 
        RRT::BoundingBox(-182.88, -182.88, 182.88, 182.88),
        10, 20, 0.4, 10, 16000,
        std::vector<RRT::Obstacle> {
            RRT::Obstacle::fromRectBottomLeft(44*2.54, -30*2.54, 10*2.54, 50*2.54),
            RRT::Obstacle::fromRectBottomLeft(-15 * 2.54, -15 * 2.54, 30 * 2.54, 30 * 2.54),
            RRT::Obstacle::fromRectBottomLeft(-51 * 2.54, -27 * 2.54, 10 * 2.54, 50 * 2.54),
        }
    );

    int i = 0;
    while (i < 16000) {
        g.iterate();
        i++;
    }


    std::ofstream outFile("scripts\\out.txt");
    for (auto obstacle : g.obstacles) {
        outFile << obstacle.polygon[0].x << " " << obstacle.polygon[0].y << std::endl;
        outFile << obstacle.polygon[1].x << " " << obstacle.polygon[1].y << std::endl;
        outFile << obstacle.polygon[2].x << " " << obstacle.polygon[2].y << std::endl;
        outFile << obstacle.polygon[3].x << " " << obstacle.polygon[3].y << std::endl;
    }

    for (const auto& node : g.allNodes) {
        std::cout << "a";
        auto parent = node->parent;
        if (parent != nullptr) {
            outFile << parent->point.x << " " << parent->point.y << " ";
        }
        outFile << node->point.x << " " << node->point.y << std::endl; 
    }
    /*
    auto node = g.optimalNodeNearGoal().get();
    while (node->getParentRaw() != nullptr) {
        auto parent = node->getParentRaw();
        outFile << parent->point.x << " " << parent->point.y << " ";
        outFile << node->point.x << " " << node->point.y << std::endl;

        node = node->getParentRaw();
    }*/

    outFile.close();

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