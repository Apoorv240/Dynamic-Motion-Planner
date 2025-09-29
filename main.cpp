#include <iostream>
#include <fstream>
#include <memory>
#include <string>

#include "math/QuinticSpline.hpp"
#include "planning/Trajectory.hpp"
#include "planning/rrt/rrt.hpp"

int main() {
    RRT::Generator g(
        RRT::Point(0,0), 
        RRT::Point(900, 900), 
        RRT::BoundingBox(-1000, -1000, 1000, 1000), 
        50, 0.4, 4000,
        std::vector<RRT::Obstacle> {
            RRT::Obstacle(
                std::vector<RRT::Point>{RRT::Point(300, 0), RRT::Point(500, 0), RRT::Point(500, 500), RRT::Point(300, 500)}
            )
        }
    );
    g.iterate();
    g.iterate();
    g.iterate();

    int i = 0;
    while (i < 8000) {
        g.iterate();
        i++;
    }


    std::ofstream outFile("scripts/out.txt");
    for (const auto& node : g.allNodes) {
        auto parent = node->getParentRaw();
        if (parent != nullptr) {
            outFile << parent->point.x << " " << parent->point.y << " ";
        }
        outFile << node->point.x << " " << node->point.y << std::endl; 
    }

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