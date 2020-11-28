#include "object_follower_move_base.hpp"

constexpr double SERVER_WAIT_DURATION = 5.0;

namespace Follower {

MoveBaseFollower::MoveBaseFollower() {}

auto MoveBaseFollower::follow() -> void {
  if (!enable_following_)
    return;
  try {
    checkTf();
    tfStamped pose = getTf();
    setGoalTf(pose);
    sendGoal(tfToGoal(pose));
    showGoalState();
  } catch (tf2::LookupException &ex) {
    ROS_WARN("Object frame not found: %s", ex.what());
  } catch (tf2::TimeoutException &ex) {
    ROS_WARN("Object frame lookup exceed it's time limit : %s", ex.what());
  } catch (tf2::ConnectivityException &ex) {
    ROS_WARN("Object frame not connected to base frame !: %s", ex.what());
  } catch (...) {
    ROS_ERROR("Error, can't send goal");
  }
}

auto MoveBaseFollower::sendGoal(const MoveBaseGoal &goal) noexcept -> void {
  while (!move_base_client_.waitForServer(ros::Duration(SERVER_WAIT_DURATION))) {
    ROS_WARN("Waiting for the move_base action server to come up");
  }

  ROS_INFO("Sending goal");
  move_base_client_.sendGoal(goal);
  move_base_client_.waitForResult(ros::Duration(0.0));
}

auto MoveBaseFollower::showGoalState() const noexcept -> void {
  if (move_base_client_.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
    ROS_INFO("Goal Succeeded");
  else
    ROS_WARN("Goal Failed");
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

