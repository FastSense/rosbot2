#pragma once
#include "object_follower_2d.hpp"

#include <actionlib/client/simple_action_client.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <move_base_msgs/MoveBaseGoal.h>

namespace Follower {

using MoveBaseGoal = move_base_msgs::MoveBaseGoal;
using MoveBaseClient = actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction>;

class MoveBaseFollower : public ObjectFollower2d {
public:
  MoveBaseFollower();
  void follow() final;
  ~MoveBaseFollower() = default;

private:
  void sendGoal(const MoveBaseGoal &goal) noexcept;
  MoveBaseGoal tfToGoal(const tfStamped &pose);

private:
  MoveBaseClient move_base_client_ = MoveBaseClient("move_base", true);
};

}; // namespace Follower
