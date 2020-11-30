#pragma once
#include "object_follower_2d.hpp"
#include <tf2_ros/transform_broadcaster.h>

namespace Follower {

class TfBroadcaster : public ObjectFollower2d {
public:
  TfBroadcaster();
  virtual ~TfBroadcaster() = default;

protected:
  virtual auto follow() -> void override;

private:
  auto broadcast(const tfStamped &pose) -> void;

private:
  tf2_ros::TransformBroadcaster tf_broadcaster_;

protected:
  std::string goal_frame_ = "goal_frame";
};

}; // namespace Follower
