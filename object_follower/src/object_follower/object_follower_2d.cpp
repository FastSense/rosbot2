#include "object_follower_2d.hpp"
#include "geometry_msgs/Vector3Stamped.h"

namespace Follower {

ObjectFollower2d::ObjectFollower2d() {
  bool state = false;
  while (!state)
    try {
      current_position_ = getTf();
      setGoalTf(current_position_);
      state = true;
    } catch (...) {
      exceptionFilter();
    }
}

void ObjectFollower2d::setGoalTf(tfStamped &pose) const {
  double yaw = getYawFromQuaternion(pose.transform.rotation);
  Vector3 &translation = pose.transform.translation;

  setGoalTranslation(translation, yaw);
  setGoalRotation(pose, yaw);
}

double ObjectFollower2d::getYawFromQuaternion(const Quaternion &q) const noexcept {
  double siny_cosp = 2.0 * (q.w * q.z + q.x * q.y);
  double cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z);
  double yaw = std::atan2(siny_cosp, cosy_cosp);
  return yaw;
}

void ObjectFollower2d::setGoalRotation(tfStamped &pose, const double yaw) const {
  tf2::Quaternion q;
  q.setRPY(0, 0, M_PI + yaw);

  pose.transform.rotation.w = q.w();
  pose.transform.rotation.x = q.x();
  pose.transform.rotation.y = q.y();
  pose.transform.rotation.z = q.z();
}

void ObjectFollower2d::setGoalTranslation(Vector3 &pt, const double yaw) const {
  const double &r = goal_dist_from_obj_;
  pt.x = pt.x + r * cos(yaw);
  pt.y = pt.y + r * sin(yaw);
  pt.z = 0;
}

}; // namespace Follower

