#include "GoalGenerator.hpp"

GoalGenerator::GoalGenerator() {
  tf_listener_ = std::make_unique<tf2_ros::TransformListener>(tf_buffer_);
}

geometry_msgs::TransformStamped GoalGenerator::lookupTransform() {
  static ros::Time tf_oldness;

  tf_oldness = ros::Time(ros::Time::now());
  geometry_msgs::TransformStamped tf_pose;
  tf_pose = tf_buffer_.lookupTransform(base_frame_, object_frame_, tf_oldness, tf_wait_);
  return tf_pose;
}
