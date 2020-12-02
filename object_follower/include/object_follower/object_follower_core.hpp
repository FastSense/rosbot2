#pragma once

#include "ros/time.h"
#include "tf2_ros/transform_listener.h"
#include <ros/ros.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

#include <ros/service_server.h>
#include <std_srvs/SetBool.h>

namespace Follower {

using tfStamped = geometry_msgs::TransformStamped;
using tfListener = tf2_ros::TransformListener;

using Request = std_srvs::SetBool::Request;
using Response = std_srvs::SetBool::Response;

using QuaternionTf = tf2::Quaternion;
using Vector3Tf = tf2::Vector3;

struct PoseTf {
  Vector3Tf translation;
  QuaternionTf quaternion;
};

class ObjectFollower {
public:
  ObjectFollower();
  virtual auto start() -> void = 0;
  ~ObjectFollower() = default;

protected:
  virtual auto follow() -> void = 0;
  virtual auto sleep() -> void = 0;

  auto getTf() const -> tfStamped;
  auto enableFollowingCb(Request &req, Response &res) -> bool;

  auto updatePoseIfGood(const tfStamped &pose) -> bool;
  auto isNewGoalGood(const tfStamped &pose) const -> bool;
  auto getTfPoseFromMsg(const tfStamped &pose) const -> PoseTf;
  auto setCurrentPosition(const tfStamped &pose) -> void;

  auto exceptionFilter() const -> void;
  tfStamped current_position_;

private:
  auto setParams() -> void;

protected:
  std::string base_frame_;
  std::string object_frame_;

  bool following_enabled_ = true;
  double goal_dist_from_obj_;

  double range_diff_to_set_new_pose_;
  double angle_diff_to_set_new_pose_;

  double tf_wait_value_;
  ros::Duration tf_wait_;
  ros::NodeHandle nh_;
  ros::NodeHandle pnh_;

private:
  std::unique_ptr<tfListener> tf_listener_;
  tf2_ros::Buffer tf_buffer_;
  ros::ServiceServer service_enable_following;
};

}; // namespace Follower
