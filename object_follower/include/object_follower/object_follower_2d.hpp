#pragma once
#include "object_follower_core.hpp"

#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Quaternion.h>
#include <geometry_msgs/Vector3.h>

namespace Follower {

using PoseStamped = geometry_msgs::PoseStamped;
using Vector3 = geometry_msgs::Vector3;
using Quaternion = geometry_msgs::Quaternion;

/**
 * @brief Abstract Class extending ObjectFollower class with methods for setting goals on 2d space
 * frame names etc.
 */
class ObjectFollower2d : public ObjectFollower {
public:
  ObjectFollower2d();
  ~ObjectFollower2d() = default;

protected:
  /**
   * param[in] pose Pose which be transformed to goal pose
   */
  void setGoalTf(tfStamped &pose) const;

  /**
   * param[in] point Position of the object which transforms to goal
   * param[in] angle on which object rotated
   */
  void setGoalTranslation(Vector3 &point, const double yaw) const;

  /**
   * param[in] pose Position of the goal which rotates to goal
   * param[in] angle on which object rotated
   */
  void setGoalRotation(tfStamped &pose, const double yaw) const;

  double getYawFromQuaternion(const Quaternion &q) const noexcept;
};

}; // namespace Follower
