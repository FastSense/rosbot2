#pragma once

#include "ros/time.h"
#include "tf2_ros/transform_listener.h"
#include <geometry_msgs/TransformStamped.h>
#include <ros/ros.h>

#include <ros/service_server.h>
#include <std_srvs/SetBool.h>

namespace Follower {

using tfStamped = geometry_msgs::TransformStamped;
using tfListener = tf2_ros::TransformListener;
using Request = std_srvs::SetBool::Request;
using Response = std_srvs::SetBool::Response;

class ObjectFollower {
public:
  ObjectFollower();
  virtual auto start() -> void = 0;
  virtual ~ObjectFollower() = default;

protected:
  virtual auto follow() -> void = 0;
  virtual auto sleep() -> void = 0;

  auto checkTf() const -> void;
  auto getTf() const -> tfStamped;

  auto enableFollowingCb(Request &req, Response &res) -> bool;

private:
  auto setParams() -> void;

protected:
  std::string base_frame_ = "map";
  std::string object_frame_ = "object";

  bool enable_following_ = true;
  double goal_dist_from_obj_ = 1.0;

  double range_diff_to_set_new_pose_ = 1.0;
  double angle_diff_to_set_new_pose_ = 20.0;
  double max_dist_to_obj_ = 5.0;

  ros::Time tf_oldness_ = ros::Time(0);
  ros::NodeHandle nh_;
  ros::NodeHandle pnh_;
  tfStamped current_position_;

private:
  std::unique_ptr<tfListener> tf_listener_;
  tf2_ros::Buffer tf_buffer_;
  ros::ServiceServer service_enable_following;
};

}; // namespace Follower
