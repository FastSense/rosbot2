#include "GoalGeneratorNearest.hpp"

void GoalGeneratorNearest::evalGoal(geometry_msgs::TransformStamped &pose) {
  geometry_msgs::Vector3 &translation = pose.transform.translation;
  geometry_msgs::Quaternion &rotation = pose.transform.rotation;
}

