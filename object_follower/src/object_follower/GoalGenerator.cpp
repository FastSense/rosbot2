#include "GoalGenerator.hpp"

GoalGenerator::GoalGenerator() {
  tf_listener_ = std::make_unique<tf2_ros::TransformListener>(tf_buffer_);
}

geometry_msgs::TransformStamped GoalGenerator::lookupTransform(std::string source_frame, std::string target_frame) {
  ros::Time tf_oldness = ros::Time::now();
  geometry_msgs::TransformStamped tf_pose;
  tf_pose = tf_buffer_.lookupTransform(source_frame, target_frame, tf_oldness, tf_wait_);
  return tf_pose;
}

