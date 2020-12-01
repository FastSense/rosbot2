#include "object_follower_tf_broadcaster.hpp"

namespace Follower {

TfBroadcaster::TfBroadcaster() {
  pnh_.param<std::string>("goal_frame_", goal_frame_, "goal_to_follow");
}

auto TfBroadcaster::follow() -> void {
  if (!following_enabled_)
    return;

  try {
    auto pose_tf = getTf();
    setGoalTf(pose_tf);
    if (!updatePoseIfGood(pose_tf))
      return;
    broadcast(pose_tf);
  } catch (...) {
    exceptionFilter();
  }
}

auto TfBroadcaster::broadcast(tfStamped &pose) -> void {
  pose.child_frame_id = goal_frame_;
  tf_broadcaster_.sendTransform(pose);
}

}; // namespace Follower
