#include "object_follower_move_base.hpp"

constexpr double SERVER_WAIT_DURATION = 2.0;

namespace Follower {

MoveBaseFollower::MoveBaseFollower() {}

void MoveBaseFollower::follow() {
  if (!following_enabled_)
    return;

  tfStamped pose = getTf();
  setGoalTf(pose);
  if (!updatePoseIfConsidered(pose))
    return;
  sendGoal(tfToGoal(pose));
}

void MoveBaseFollower::sendGoal(const MoveBaseGoal &goal) {
  while (!move_base_client_.waitForServer(ros::Duration(SERVER_WAIT_DURATION))) {
    ROS_WARN("Waiting for the move_base action server to come up");
  }

  ROS_INFO("Sending goal");
  move_base_client_.sendGoal(goal);
}

MoveBaseGoal MoveBaseFollower::tfToGoal(const tfStamped &pose) const {
  MoveBaseGoal converted_pose;

  converted_pose.target_pose.pose.orientation.w = pose.transform.rotation.w;
  converted_pose.target_pose.pose.orientation.x = pose.transform.rotation.x;
  converted_pose.target_pose.pose.orientation.y = pose.transform.rotation.y;
  converted_pose.target_pose.pose.orientation.z = pose.transform.rotation.z;

  converted_pose.target_pose.pose.position.x = pose.transform.translation.x;
  converted_pose.target_pose.pose.position.y = pose.transform.translation.y;
  converted_pose.target_pose.pose.position.z = pose.transform.translation.z;

  converted_pose.target_pose.header.frame_id = pose.header.frame_id;
  converted_pose.target_pose.header.seq = pose.header.seq;
  converted_pose.target_pose.header.stamp = pose.header.stamp;

  return converted_pose;
}

}; // namespace Follower
