#include "object_follower_tf_broadcaster.hpp"

namespace Follower {

TfBroadcaster::TfBroadcaster() {
  pnh_.param<std::string>("goal_frame_", goal_frame_, "goal_to_follow");
}

void TfBroadcaster::follow() {
  if (!following_enabled_)
    return;

  try {
    auto pose_tf = getTf();
    setGoalTf(pose_tf);
    if (updatePoseIfGood(pose_tf))
      broadcast(pose_tf);
  } catch (...) {
    exceptionFilter();
  }
}

void TfBroadcaster::broadcast(tfStamped &pose) {
  pose.child_frame_id = goal_frame_;
  tf_broadcaster_.sendTransform(pose);
}

}; // namespace Follower
