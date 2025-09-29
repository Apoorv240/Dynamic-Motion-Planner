#pragma once

#include "../math/QuinticSpline.hpp"
#include "../math/geometry.hpp"

#include <memory>

enum class TrajectoryType {
    LINE, SPLINE
};

class TrajectorySegment {
public:
    Pose2d _startPose;
    Pose2d _endPose;

    TrajectorySegment()
        : previous(nullptr), next(nullptr)
    {}

    TrajectorySegment(const Pose2d startPose, const Pose2d endPose, const TrajectoryType type) 
        : _startPose(startPose), _endPose(endPose), previous(nullptr), next(nullptr)
    {}

    TrajectorySegment* previous;
    std::unique_ptr<TrajectorySegment> next;
};

class Trajectory {
public:
    std::unique_ptr<TrajectorySegment> head_;
    TrajectorySegment* tail_;
    Pose2d startPose_;

    Trajectory()
        : tail_(nullptr), head_(nullptr), startPose_()
    {}

    Trajectory& start(const Pose2d pose);
    Trajectory& lineTo(const Pose2d pose);
};