#include "GoalPublisherTf.hpp"

void GoalPublisherTf::sendGoal(geometry_msgs::TransformStamped &pose) {
  pose.child_frame_id = goal_frame_;
  tf_broadcaster_.sendTransform(pose);
}
