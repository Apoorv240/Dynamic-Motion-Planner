#include <iostream>
#include <fstream>
#include <memory>
#include <string>
#include <chrono>

#include "../planning/rrt/rrt.hpp"
#include "../planning/spline/spline.hpp"
#include "../planning/gvf/gvf.hpp"
#include "../planning/planner.hpp"

int main() {
    auto startTime = std::chrono::high_resolution_clock::now();    
    // Vec2d start = Vec2d(-150, 0);
    // RRT::Generator g(
    //     start, 
    //     Vec2d(75,0),//Vec2d(170, 0), 
    //     RRT::BoundingBox(-182.88, -182.88, 182.88, 182.88),
    //     20, 0.4, 2, 2000,
    //     std::vector<RRT::Obstacle> {
    //         RRT::Obstacle::fromRectBottomLeft(44*2.54, -30*2.54, 10*2.54, 50*2.54),
    //         RRT::Obstacle::fromRectBottomLeft(-15 * 2.54, -15 * 2.54, 30 * 2.54, 30 * 2.54),
    //         RRT::Obstacle::fromRectBottomLeft(-51 * 2.54, -27 * 2.54, 10 * 2.54, 50 * 2.54),
    //     }
    // );

    // //int iterations = g.iterateUntilPathFound(5000);
    // int iterations = 2000;
    // g.iterateIterations(iterations);

    // std::vector<RRT::Node*> path = g.getOptimalPath();
    
    // Spline sg(path, 3);
    // sg.generate(1e-5);

    // auto points = sg.sampleSpline(50);

    Pose2d startPose = Pose2d(-150, 0);
    Pose2d endPose = Pose2d(75, 0);

    std::vector<RRT::Obstacle> obstacles {
        RRT::Obstacle::fromRectBottomLeft(44*2.54, -30*2.54, 10*2.54, 50*2.54),
        RRT::Obstacle::fromRectBottomLeft(-15 * 2.54, -15 * 2.54, 30 * 2.54, 30 * 2.54),
        RRT::Obstacle::fromRectBottomLeft(-51 * 2.54, -27 * 2.54, 10 * 2.54, 50 * 2.54),
    };

    Planner planner(startPose, endPose, RRT::BoundingBox(-182.88, -182.88, 182.88, 182.88), obstacles);
    planner.genGlobalPath();

    auto endTime = std::chrono::high_resolution_clock::now();

    std::chrono::duration<double, std::milli> duration = endTime - startTime;
    std::cout << "Iterated for " << duration.count() << " milliseconds" << std::endl;
    std::cout << "Iterated " << planner.RRT_ITERATIONS << " times" << std::endl;
    // std::cout << g.nodeManager.size << " Nodes generated" << std::endl;

    // std::cout << "t: " << sg.numT() << std::endl;
    // std::cout << "knots: " << sg.numKnots() << std::endl;
    // std::cout << "control: " << sg.numControlPoints() << std::endl << std::endl;

    std::cout << "Generating files..." << std::endl << std::flush;

    auto points = planner.globalPath.sampleSpline(50);

    std::ofstream outFile("scripts\\outAll.txt");
    for (auto obstacle : obstacles) {
        outFile << obstacle.polygon[0].x() << " " << obstacle.polygon[0].y() << std::endl;
        outFile << obstacle.polygon[1].x() << " " << obstacle.polygon[1].y() << std::endl;
        outFile << obstacle.polygon[2].x() << " " << obstacle.polygon[2].y() << std::endl;
        outFile << obstacle.polygon[3].x() << " " << obstacle.polygon[3].y() << std::endl;
    }

    for (const auto& node : planner.rrt.nodeManager.nodes) {
        auto parent = node->parent;
        if (parent != nullptr) {
            outFile << parent->point.x() << " " << parent->point.y() << " ";
        }
        outFile << node->point.x() << " " << node->point.y() << std::endl; 
    }

    outFile.close();

    std::ofstream outFile2("scripts\\outPath.txt");
    {
        for (auto obstacle : obstacles) {
            outFile2 << obstacle.polygon[0].x() << " " << obstacle.polygon[0].y() << std::endl;
            outFile2 << obstacle.polygon[1].x() << " " << obstacle.polygon[1].y() << std::endl;
            outFile2 << obstacle.polygon[2].x() << " " << obstacle.polygon[2].y() << std::endl;
            outFile2 << obstacle.polygon[3].x() << " " << obstacle.polygon[3].y() << std::endl;
        }
        
        std::vector<RRT::Node*> path = planner.rrt.getOptimalPath();

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
        for (auto obstacle : obstacles) {
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

    std::ofstream outFile4("scripts\\outGVF.txt");
    {
        for (auto obstacle : obstacles) {
            outFile4 << obstacle.polygon[0].x() << " " << obstacle.polygon[0].y() << std::endl;
            outFile4 << obstacle.polygon[1].x() << " " << obstacle.polygon[1].y() << std::endl;
            outFile4 << obstacle.polygon[2].x() << " " << obstacle.polygon[2].y() << std::endl;
            outFile4 << obstacle.polygon[3].x() << " " << obstacle.polygon[3].y() << std::endl;
        }

        for (int x = -180; x < 180; x+=15) {
            for (int y = -180; y < 180; y+=15) {
                auto v = planner.getDirectionVectorAt(Eigen::Vector2d(x, y));
                outFile4 << x << " " << y << " " << v.x() << " " << v.y() << std::endl;
            }
        }

        for (auto point : points) {
            auto v = planner.getDirectionVectorAt(point);
            outFile4 << point.x() << " " << point.y() << " " << v.x() << " " << v.y() << std::endl;
        }
    }
    outFile4.close();
}