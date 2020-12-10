#include "GoalPublisherMoveBase.hpp"
#include "Converter.hpp"

constexpr double SERVER_WAIT_DURATION = 2.0;

void GoalPublisherMoveBase::sendGoal(geometry_msgs::TransformStamped &pose) {
  auto goal = Converter::tfToMoveBaseGoal(pose);
  while (not move_base_client_.waitForServer(ros::Duration(SERVER_WAIT_DURATION))) {
    ROS_WARN("Waiting for the move_base action server to come up");
  }

  ROS_INFO("Sending goal");
  move_base_client_.sendGoal(goal);
}
