#include "Trajectory.hpp"

Trajectory& Trajectory::start(const Pose2d pose) {
    startPose_ = pose;

    return *this;
}

Trajectory& Trajectory::lineTo(const Pose2d pose) {
    auto newTraj = std::make_unique<TrajectorySegment>(tail_ ? tail_->_endPose : startPose_, pose, TrajectoryType::LINE);

    if (!head_) {
        head_ = std::move(newTraj);
        tail_ = head_.get();
    } else {
        newTraj->previous = tail_;
        tail_->next = std::move(newTraj);
        tail_ = tail_->next.get();
    }

    return *this;
}