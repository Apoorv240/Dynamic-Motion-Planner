#include <iostream>
#include <fstream>
#include <memory>
#include <string>
#include <chrono>

#include <windows.h>

#include "planning/rrt/rrt.hpp"
#include "planning/spline/spline.hpp"
#include "planning/gvf/gvf.hpp"
#include "planning/planner.hpp"
#include "simulation/robot.hpp"
#include "math/angle.hpp"

inline std::vector<Eigen::Vector2d> makeSquareVertices(double L) {
    double h = L / 2.0; // half side
    return {
        {-h, -h},
        { h, -h},
        { h,  h},
        {-h,  h}
    };
}

// 2D rotation matrix for angle theta (radians)
inline Eigen::Matrix2d rotationMatrix2D(double theta) {
    Eigen::Matrix2d R;
    R << std::cos(theta), -std::sin(theta),
         std::sin(theta),  std::cos(theta);
    return R;
}

// Rotate and translate square vertices around center
inline std::vector<Eigen::Vector2d> rotateAndTranslateSquare(
    const std::vector<Eigen::Vector2d>& vertices,
    double theta_rad,
    const Eigen::Vector2d& center = Eigen::Vector2d::Zero()
) {
    Eigen::Matrix2d R = rotationMatrix2D(theta_rad);
    std::vector<Eigen::Vector2d> result;
    result.reserve(vertices.size());

    for (const auto& v : vertices)
        result.push_back(R * v + center); // rotate then translate

    return result;
}

int main() {
    auto startTime = std::chrono::high_resolution_clock::now();

    Pose2d startPose = Pose2d(-150, 150, angle_utils::deg2rad(90));
    Pose2d endPose = Pose2d(150, -150, angle_utils::deg2rad(-90));

    std::vector<RRT::Obstacle> obstacles {
        RRT::Obstacle::fromRectBottomLeftExpanded(44*2.54, -30*2.54, 10*2.54, 50*2.54, 9 * 2.54),
        RRT::Obstacle::fromRectBottomLeftExpanded(-15 * 2.54, -15 * 2.54, 30 * 2.54, 30 * 2.54, 9 * 2.54),
        RRT::Obstacle::fromRectBottomLeftExpanded(-51 * 2.54, -27 * 2.54, 10 * 2.54, 50 * 2.54, 9 * 2.54),
        RRT::Obstacle::fromRectBottomLeft(44*2.54, -30*2.54, 10*2.54, 50*2.54),
        RRT::Obstacle::fromRectBottomLeft(-15 * 2.54, -15 * 2.54, 30 * 2.54, 30 * 2.54),
        RRT::Obstacle::fromRectBottomLeft(-51 * 2.54, -27 * 2.54, 10 * 2.54, 50 * 2.54),
    };

    Planner planner(startPose, endPose, RRT::BoundingBox(-182.88, -182.88, 182.88, 182.88), obstacles);
    planner.genGlobalPath();

    auto endTime = std::chrono::high_resolution_clock::now();

    std::chrono::duration<double, std::milli> duration = endTime - startTime;
    std::cout << "Generated for " << duration.count() << " milliseconds" << std::endl;


    Robot simRobot(startPose);
    auto robotSquare = makeSquareVertices(18*2.54);

    FILE* gp = _popen("gnuplot -persistent", "w");
    if (!gp) return 1;

    // Set up plot
    fprintf(gp, "set xrange [-182.88:182.88]\nset yrange [-182.88:182.88]\n");
    fprintf(gp, "set xlabel 'x'\nset ylabel 'y'\n");
    fprintf(gp, "set key off\n");
    fprintf(gp, "set size ratio -1\n");

    // Animate robot
    while (!simRobot.atTarget(endPose.pos, 3)) {
        // 1. Build the plot command for all obstacles + path + robot
        fprintf(gp, "plot ");
        for (size_t i = 0; i < obstacles.size(); ++i) {
            fprintf(gp, "'-' with lines lt rgb 'blue' lw 2");
            if (i != obstacles.size() - 1) fprintf(gp, ", "); // comma between obstacles
        }
        if (!obstacles.empty()) fprintf(gp, ", "); // comma before path if there are obstacles
        fprintf(gp, "'-' with lines lt rgb 'black' lw 2, "); // path
        fprintf(gp, "'-' with lines lt rgb 'red' lw 2, "); // robot square
        fprintf(gp, "'-' with points pt 7 ps 2 lc rgb 'red'\n"); // robot

        // 2. Send obstacle data
        for (auto &obstacle : obstacles) {
            for (auto &pt : obstacle.polygon) {
                fprintf(gp, "%f %f\n", pt.x(), pt.y());
            }
            // Optionally close the polygon by repeating the first point
            fprintf(gp, "%f %f\n", obstacle.polygon[0].x(), obstacle.polygon[0].y());
            fprintf(gp, "e\n");
        }

        // 3. Send global path data
        auto points = planner.globalPath.sampleSpline(100);
        for (auto &pt : points) {
            fprintf(gp, "%f %f\n", pt.x(), pt.y());
        }
        fprintf(gp, "e\n");

        auto direction = planner.getDirectionVectorAt(
            Eigen::Vector2d(simRobot.pose.pos.x(), simRobot.pose.pos.y()));

        auto displayDirection = Eigen::Vector2d(simRobot.pose.pos.x(), simRobot.pose.pos.y()) + direction*10;

        // Display robot square
        auto rotatedRobotSquare = rotateAndTranslateSquare(robotSquare, angle_utils::angle(simRobot.pose.vel), simRobot.pose.pos);
        for (auto &pt : rotatedRobotSquare) {
            fprintf(gp, "%f %f\n", pt.x(), pt.y());
        }
        // Optionally close the polygon by repeating the first point
        fprintf(gp, "%f %f\n", rotatedRobotSquare[0].x(), rotatedRobotSquare[0].y());
        fprintf(gp, "e\n");

        // 4. Send robot position
        fprintf(gp, "%f %f\n", simRobot.pose.pos.x(), simRobot.pose.pos.y());
        fprintf(gp, "e\n");

        fprintf(gp, "unset arrow\n"); // remove previous arrow (important)
        fprintf(gp, "set arrow from %f,%f to %f,%f lw 2 lc rgb 'red' head filled size screen 0.02,15,45\n",
        simRobot.pose.pos.x(), simRobot.pose.pos.y(), displayDirection.x(), displayDirection.y());

        fflush(gp);

        // 5. Update robot (simulation)
        simRobot.update(direction, 10);
    }

    _pclose(gp);
}