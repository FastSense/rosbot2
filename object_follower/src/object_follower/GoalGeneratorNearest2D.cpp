#include "GoalGeneratorNearest2D.hpp"
#include "Converter.hpp"

#include <cmath>

void GoalGeneratorNearest2D::evalGoal(geometry_msgs::TransformStamped &pose) {
  const double &r = goal_dist_from_obj_;
  geometry_msgs::Vector3 &translation = pose.transform.translation;
  geometry_msgs::Quaternion &rotation = pose.transform.rotation;

  double alpha = atan2(translation.y, translation.x);

  translation.x = translation.x - r * cos(alpha);
  translation.y = translation.y - r * sin(alpha);
  translation.z = 0;

  Converter::setQuaternionRotation(rotation, RPY{0, 0, alpha});
}

