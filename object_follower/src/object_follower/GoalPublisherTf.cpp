#include "GoalPublisherTf.hpp"

void GoalPublisherTf::sendGoal(geometry_msgs::TransformStamped &pose) {
  pose.header.frame_id = base_frame_;
  pose.child_frame_id = goal_frame_;
  tf_broadcaster_.sendTransform(pose);
}
