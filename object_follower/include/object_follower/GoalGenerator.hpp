#pragma once

#include "geometry_msgs/TransformStamped.h"
#include "tf2_ros/transform_listener.h"

class GoalGenerator {
public:
  GoalGenerator();
  virtual ~GoalGenerator() = default;

  virtual void evalGoalTf(geometry_msgs::TransformStamped &pose);

  geometry_msgs::TransformStamped lookupTransform();

  double tf_wait_value_;

  /// Require init in ObjectFollowerCore
  ros::Duration tf_wait_;

  std::string object_frame_;
  std::string base_frame_;

  double goal_dist_from_obj_;

protected:
  std::unique_ptr<tf2_ros::TransformListener> tf_listener_;
  tf2_ros::Buffer tf_buffer_;
};
