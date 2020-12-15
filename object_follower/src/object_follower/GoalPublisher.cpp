#include "GoalPublisher.hpp"

GoalPublisher::GoalPublisher() : rate_(RATE_VALUE) {
  tf_listener_ = std::make_unique<tf2_ros::TransformListener>(tf_buffer_);
}

geometry_msgs::TransformStamped GoalPublisher::lookupTransform() {
  ros::Time tf_oldness = ros::Time::now();
  geometry_msgs::TransformStamped tf_pose;
  tf_pose = tf_buffer_.lookupTransform(map_frame_, object_frame_, tf_oldness, tf_wait_);
  return tf_pose;
}
