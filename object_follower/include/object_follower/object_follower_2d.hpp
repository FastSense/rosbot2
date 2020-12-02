#pragma once
#include "object_follower_core.hpp"

#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Quaternion.h>
#include <geometry_msgs/Vector3.h>

namespace Follower {

using PoseStamped = geometry_msgs::PoseStamped;
using Vector3 = geometry_msgs::Vector3;
using Quaternion = geometry_msgs::Quaternion;

class ObjectFollower2d : public ObjectFollower {
public:
  ObjectFollower2d();
  ~ObjectFollower2d() = default;

protected:
  auto setGoalTf(tfStamped &pose) const -> void;

  auto setGoalTranslation(Vector3 &point, const double yaw) const -> void;
  auto setGoalRotation(tfStamped &pose, const double yaw) const -> void;

  auto getYawFromQuaternion(const Quaternion &q) const noexcept -> double;
};

}; // namespace Follower
