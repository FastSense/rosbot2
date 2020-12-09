#pragma once
#include "objf_core.hpp"

#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Quaternion.h>
#include <geometry_msgs/Vector3.h>

namespace Follower {

using PoseStamped = geometry_msgs::PoseStamped;
using Vector3 = geometry_msgs::Vector3;
using Quaternion = geometry_msgs::Quaternion;

struct RPY {
  double roll;
  double pitch;
  double yaw;
};

/**
 * @brief Abstract Class extending ObjectFollower class with methods for setting goals on 2d space
 * frame names etc.
 */
class ObjectFollower2d : public ObjectFollowerCore {
public:
  /// empty
  ObjectFollower2d();
  ~ObjectFollower2d() = default;

protected:
  /**
   * param[in, out] pose Pose which be transformed to goal pose
   */
  void setGoalTf(tfStamped &pose) const;

  RPY getRPYfromQuaternion(const Quaternion &rotation) const;

  /**
   * param[in, out] point Position of the object which transforms to goal
   * param[in] rpy object roll pitch and yaw
   */
  void setGoalTranslation(Vector3 &translation, const double angle) const;

  /**
   * param[in, out] pose Position of the goal which rotates to goal
   * param[in] angle on which object rotated
   */
  void setGoalRotation(Quaternion &rotation, const double angle) const;

};

}; // namespace Follower
