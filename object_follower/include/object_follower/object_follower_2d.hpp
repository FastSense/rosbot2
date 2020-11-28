#pragma once
#include "object_follower_core.hpp"

#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Quaternion.h>
#include <geometry_msgs/Vector3.h>

namespace Follower {

using PoseStamped = geometry_msgs::PoseStamped;
using Vector3 = geometry_msgs::Vector3;
using QuaternionMsg = geometry_msgs::Quaternion;
using QuaternionTf = tf2::Quaternion;

class ObjectFollower2d : public ObjectFollower {
public:
  ObjectFollower2d();
  virtual auto follow() -> void override = 0;
  virtual ~ObjectFollower2d() = default;

protected:
  auto setGoalTf(tfStamped &pose) -> void;
  auto setGoalTranslation(Vector3 &point, const double yaw) -> void;
  auto setGoalRotation(tfStamped &pose, const double yaw) -> void;
  auto getYawFromQuaternion(const QuaternionMsg &q) noexcept -> double;
};

}; // namespace Follower
