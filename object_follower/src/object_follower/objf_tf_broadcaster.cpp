#include "objf_tf_broadcaster.hpp"

namespace Follower {

TfBroadcasterFollower::TfBroadcasterFollower() {
  pnh_.param<std::string>("goal_frame", goal_frame_, "goal_to_follow");
}

void TfBroadcasterFollower::follow() {
  if (!following_enabled_)
    return;

  auto pose_tf = getTf();
  setGoalTf(pose_tf);
  if (updatePoseIfConsiderable(pose_tf))
    broadcast(pose_tf);
}

void TfBroadcasterFollower::broadcast(tfStamped &pose) {
  pose.header.frame_id = goal_base_frame_;
  pose.child_frame_id = goal_frame_;
  tf_broadcaster_.sendTransform(pose);
}

}; // namespace Follower
