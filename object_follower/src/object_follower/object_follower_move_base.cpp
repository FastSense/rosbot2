#include "object_follower_move_base.hpp"

constexpr double SERVER_WAIT_DURATION = 2.0;

namespace Follower {

MoveBaseFollower::MoveBaseFollower() {}

auto MoveBaseFollower::follow() -> void {
  if (!following_enabled_)
    return;

  try {
    tfStamped pose = getTf();
    setGoalTf(pose);
    if (!updatePoseIfGood(pose))
      return;
    sendGoal(tfToGoal(pose));
  } catch (...) {
    exceptionFilter();
  }
} // namespace Follower

auto MoveBaseFollower::sendGoal(const MoveBaseGoal &goal) noexcept -> void {
  while (!move_base_client_.waitForServer(ros::Duration(SERVER_WAIT_DURATION))) {
    ROS_WARN("Waiting for the move_base action server to come up");
  }

  ROS_INFO("Sending goal");
  move_base_client_.sendGoal(goal);
}

auto MoveBaseFollower::tfToGoal(const tfStamped &pose) -> MoveBaseGoal {
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
