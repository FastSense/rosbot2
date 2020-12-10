#include "GoalGeneratorBaselink2D.hpp"
#include "Converter.hpp"

void GoalGeneratorBaselink2D::evalGoal(geometry_msgs::TransformStamped &pose) {
  geometry_msgs::Vector3 &translation = pose.transform.translation;
  geometry_msgs::Quaternion &rotation = pose.transform.rotation;

  auto rpy = Converter::getRPYfromQuaternion(pose.transform.rotation);

  setGoalTranslation(translation, rpy);
  setGoalRotation(rotation, rpy);
}

void GoalGeneratorBaselink2D::setGoalRotation(geometry_msgs::Quaternion &rotation, RPY rpy) const {
  tf2::Quaternion q;
  q.setRPY(0, 0, M_PI + rpy.yaw);
  rotation.w = q.w();
  rotation.x = q.x();
  rotation.y = q.y();
  rotation.z = q.z();
}

void GoalGeneratorBaselink2D::setGoalTranslation(geometry_msgs::Vector3 &pt, RPY rpy) const {
  const double &r = goal_dist_from_obj_;
  double angle;
  angle = rpy.yaw;

  pt.x = pt.x + r * cos(angle);
  pt.y = pt.y + r * sin(angle);
  pt.z = 0;
}
