#pragma once
#include "object_follower_core.hpp"

#include "tf2/LinearMath/Transform.h"
#include "tf2/transform_datatypes.h"
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Quaternion.h>
#include <geometry_msgs/Vector3.h>

namespace Follower {

using PoseStamped = geometry_msgs::PoseStamped;
using Vector3 = geometry_msgs::Vector3;
using Vector3Tf = tf2::Vector3;

using Quaternion = geometry_msgs::Quaternion;
using QuaternionTf = tf2::Quaternion;

struct PoseTf {
  Vector3Tf translation;
  QuaternionTf quaternion;
};

class ObjectFollower2d : public ObjectFollower {
public:
  ObjectFollower2d();
  virtual ~ObjectFollower2d() = default;

protected:
  auto setGoalTf(tfStamped &pose) const -> void;
  auto setGoalTranslation(Vector3 &point, const double yaw) const -> void;
  auto setGoalRotation(tfStamped &pose, const double yaw) const -> void;
  auto getYawFromQuaternion(const Quaternion &q) const noexcept -> double;

  auto updatePoseIfGood(const tfStamped &pose) -> bool;
  auto isNewGoalGood(const tfStamped &pose) const -> bool;
  auto getTfPoseFromMsg(const tfStamped &pose) const -> PoseTf;
  auto setCurrentPosition(const tfStamped &pose) -> void;

  tfStamped current_position_;
};

}; // namespace Follower
