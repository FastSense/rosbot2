#pragma once
#include "objf_2d.hpp"

#include <actionlib/client/simple_action_client.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <move_base_msgs/MoveBaseGoal.h>

namespace Follower {

using MoveBaseGoal = move_base_msgs::MoveBaseGoal;
using MoveBaseClient = actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction>;

/**
 * @brief Abstract Class extending ObjectFollower2d class with methods for sending goals to
 * move_base
 */
class MoveBaseFollower : public ObjectFollower2d {
public:
  /// empty
  MoveBaseFollower();

  void start() final;
  void sleep() final;

  /**
   * Implementation of follow method
   */
  void follow() final;

  ~MoveBaseFollower() = default;

private:
  void sendGoal(const MoveBaseGoal &goal);
  MoveBaseGoal tfToGoal(const tfStamped &pose) const;
  ros::Rate node_rate_;

private:
  MoveBaseClient move_base_client_ = MoveBaseClient("move_base", true);
};

}; // namespace Follower
