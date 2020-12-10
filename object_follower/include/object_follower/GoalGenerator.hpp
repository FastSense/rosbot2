#pragma once

#include "geometry_msgs/TransformStamped.h"
#include "tf2_ros/transform_listener.h"

constexpr double TF_WAIT_VALUE = 1.0;

class GoalGenerator {
public:
  GoalGenerator();
  virtual ~GoalGenerator() = default;
  virtual void evalGoal(geometry_msgs::TransformStamped &pose) = 0;

  geometry_msgs::TransformStamped lookupTransform();

public:
  std::string object_frame_;
  std::string base_frame_;

  double goal_dist_from_obj_;

private:
  ros::Duration tf_wait_ = ros::Duration(TF_WAIT_VALUE);
  std::unique_ptr<tf2_ros::TransformListener> tf_listener_;
  tf2_ros::Buffer tf_buffer_;
};
