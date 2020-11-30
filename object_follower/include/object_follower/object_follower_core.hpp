#pragma once

#include "ros/time.h"
#include "tf2_ros/transform_listener.h"
#include <geometry_msgs/TransformStamped.h>
#include <ros/ros.h>

namespace Follower {

using tfStamped = geometry_msgs::TransformStamped;
using tfListener = tf2_ros::TransformListener;

static constexpr int DEFAULT_OLDNESS = 0;

class ObjectFollower {
public:
  ObjectFollower();
  virtual auto follow() -> void = 0;
  virtual ~ObjectFollower() = default;

protected:
  auto checkTf() const -> void;
  auto getTf() const -> tfStamped;

protected:
  ros::Time tf_oldness_ = ros::Time(DEFAULT_OLDNESS);

  std::string base_frame_ = "map";
  std::string object_frame_ = "object";

  bool enable_following_ = true;
  double goal_dist_from_obj_ = 1.0;

  double range_diff_to_set_new_pose_ = 1.0;
  double angle_diff_to_set_new_pose_ = 20.0;
  double max_dist_to_obj_ = 5.0;

private:
  std::unique_ptr<tfListener> tf_listener_;
  tf2_ros::Buffer tf_buffer_;
  tfStamped current_position_;

};

}; // namespace Follower
