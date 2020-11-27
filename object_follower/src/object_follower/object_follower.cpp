#include "object_follower.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include <tf2_ros/transform_broadcaster.h>

constexpr double SERVER_WAIT_DURATION = 5.0;

namespace Follower {

ObjectFollower::ObjectFollower() : tf_listener_(std::make_unique<tfListener>(tf_buffer_)) {}

auto ObjectFollower::follow() -> void {
  if (send_tf_instead_)
    sendTfToFollow();
  else
    moveBaseFollow();
}

auto ObjectFollower::moveBaseFollow() -> void {
  try {
    checkTf();
    auto pose_tf = getObjTf();
    evalTfGoal(pose_tf);
    sendGoal(tfToGoal(pose_tf));
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

auto ObjectFollower::sendTfToFollow() -> void {
  static tf2_ros::TransformBroadcaster tf_broacaster;

  try {
    checkTf();
    auto pose_tf = getObjTf();
    evalTfGoal(pose_tf);
    pose_tf.child_frame_id = goal_frame_;
    tf_broacaster.sendTransform(pose_tf);
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

auto ObjectFollower::sendGoal(const MoveBaseGoal &goal) noexcept -> void {
  while (!move_base_client_.waitForServer(ros::Duration(SERVER_WAIT_DURATION))) {
    ROS_WARN("Waiting for the move_base action server to come up");
  }

  ROS_INFO("Sending goal");
  move_base_client_.sendGoal(goal);
  move_base_client_.waitForResult(ros::Duration(0.0));
}

auto ObjectFollower::showGoalState() const noexcept -> void {
  if (move_base_client_.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
    ROS_INFO("Goal Succeeded");
  else
    ROS_WARN("Goal Failed");
}

auto ObjectFollower::checkTf() const -> void {
  tf_buffer_.canTransform(base_frame_, object_frame_, tf_oldness_);
}

auto ObjectFollower::getObjTf() const -> tfStamped {
  tfStamped tf_pose;
  tf_pose = tf_buffer_.lookupTransform(base_frame_, object_frame_, tf_oldness_);
  return tf_pose;
}

auto ObjectFollower::evalTfGoal(tfStamped &pose) -> void {
  double yaw = getYawFromQuaternion(pose.transform.rotation);
  Vector3 &translation = pose.transform.translation;

  setTranslationalGoal(translation, yaw);
  setRotationalGoal(pose, yaw);
}

auto ObjectFollower::getYawFromQuaternion(const QuaternionMsg &q) noexcept -> double {
  double siny_cosp = 2.0 * (q.w * q.z + q.x * q.y);
  double cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z);
  double yaw = std::atan2(siny_cosp, cosy_cosp);
  return yaw;
}

auto ObjectFollower::setRotationalGoal(tfStamped &pose, const double yaw) -> void {
  tf2::Quaternion q;
  q.setRPY(0, 0, M_PI + yaw);

  pose.transform.rotation.w = q.w();
  pose.transform.rotation.x = q.x();
  pose.transform.rotation.y = q.y();
  pose.transform.rotation.z = q.z();
}

auto ObjectFollower::setTranslationalGoal(Vector3 &pt, const double yaw) -> void {
  const double &r = goal_dist_from_obj_;
  pt.x = pt.x + r * cos(yaw);
  pt.y = pt.y + r * sin(yaw);
  pt.z = 0;
}

auto ObjectFollower::tfToGoal(const tfStamped &pose) -> MoveBaseGoal {
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

