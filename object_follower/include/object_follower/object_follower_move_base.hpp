#pragma once
#include "object_follower_2d.hpp"

#include <move_base_msgs/MoveBaseGoal.h>
#include <actionlib/client/simple_action_client.h>
#include <move_base_msgs/MoveBaseAction.h>

namespace Follower {

using MoveBaseGoal = move_base_msgs::MoveBaseGoal;
using MoveBaseClient = actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction>;

class MoveBaseFollower : public ObjectFollower2d {
public:
  MoveBaseFollower();
  virtual auto follow() -> void override;
  virtual ~MoveBaseFollower() = default;

private:
  auto sendGoal(const MoveBaseGoal &goal) noexcept -> void;
  auto tfToGoal(const tfStamped &pose) -> MoveBaseGoal;

private:
  MoveBaseClient move_base_client_ = MoveBaseClient("move_base", true);
};

}; // namespace Follower
