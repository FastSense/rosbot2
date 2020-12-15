#pragma once

#include "geometry_msgs/TransformStamped.h"
#include "tf2_ros/transform_listener.h"

constexpr double TF_WAIT_VALUE = 1.0;

/**
 * @brief Abstract class represents algorithm for evaluating goal.
 */
class GoalGenerator {
public:
  GoalGenerator();
  virtual ~GoalGenerator() = default;
  virtual geometry_msgs::TransformStamped evalGoal() = 0;

  geometry_msgs::TransformStamped lookupTransform(std::string source_frame, std::string target_frame);

public:
  std::string map_frame_;
  std::string object_frame_;
  std::string base_frame_;

  ///  desired distance between goal and object
  double goal_dist_from_obj_;

private:
  ros::Duration tf_wait_ = ros::Duration(TF_WAIT_VALUE);
  std::unique_ptr<tf2_ros::TransformListener> tf_listener_;
  tf2_ros::Buffer tf_buffer_;
};
