#pragma once
#include "objf_2d.hpp"
#include <tf2_ros/transform_broadcaster.h>

namespace Follower {

/// Abstract Class extending ObjectFollower2d class with methods for broadcasting Tf
class TfBroadcasterFollower : public ObjectFollower2d {
public:
   /// Initialize TfBroadcasterFollower::goal_frame_ through parameter server
  TfBroadcasterFollower();
  ~TfBroadcasterFollower() = default;

protected:
  void follow() final;

private:
  void broadcast(tfStamped &pose);

private:
  tf2_ros::TransformBroadcaster tf_broadcaster_;

protected:
  std::string goal_frame_ = "goal_frame";
};

}; // namespace Follower
