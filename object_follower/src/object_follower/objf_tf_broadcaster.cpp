#include "objf_tf_broadcaster.hpp"

constexpr double NODE_RATE = 10.0;

namespace Follower {

TfBroadcasterFollower::TfBroadcasterFollower() : node_rate_(NODE_RATE) {
  pnh_.param<std::string>("goal_frame", goal_frame_, "goal_to_follow");
}

void TfBroadcasterFollower::sleep() { node_rate_.sleep(); }

void TfBroadcasterFollower::start() {
  while (ros::ok()) {
    try {
      follow();
    } catch (...) {
      exceptionFilter();
    }

    ros::spinOnce();
    sleep();
  }
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
