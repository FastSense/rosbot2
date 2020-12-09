#include "objf_2d.hpp"
#include "geometry_msgs/Vector3Stamped.h"
#include "tf2/convert.h"

namespace Follower {

ObjectFollower2d::ObjectFollower2d() {}

void ObjectFollower2d::setGoalTf(tfStamped &pose) const {
  Vector3 &translation = pose.transform.translation;
  Quaternion &rotation = pose.transform.rotation;

  auto rpy = getRPYfromQuaternion(pose.transform.rotation);

  setGoalTranslation(translation, rpy.yaw);
  setGoalRotation(rotation, rpy.yaw);
}

RPY ObjectFollower2d::getRPYfromQuaternion(const Quaternion &rotation) const {
  QuaternionTf q_tf;
  tf2::convert(rotation, q_tf);
  double r, p, y;
  tf2::Matrix3x3(q_tf).getRPY(r, p, y);

  return RPY{r, p, y};
}

void ObjectFollower2d::setGoalRotation(Quaternion &rotation, double angle) const {
  tf2::Quaternion q;
  q.setRPY(0, 0, M_PI + angle);
  rotation.w = q.w();
  rotation.x = q.x();
  rotation.y = q.y();
  rotation.z = q.z();
}

void ObjectFollower2d::setGoalTranslation(Vector3 &pt, double angle) const {
  const double &r = goal_dist_from_obj_;

  pt.x = pt.x + r * cos(angle);
  pt.y = pt.y + r * sin(angle);
  pt.z = 0;
}

}; // namespace Follower

