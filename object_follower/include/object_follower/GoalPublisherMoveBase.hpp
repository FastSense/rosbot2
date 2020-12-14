#include "GoalPublisher.hpp"

#include <actionlib/client/simple_action_client.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <move_base_msgs/MoveBaseGoal.h>

using MoveBaseGoal = move_base_msgs::MoveBaseGoal;
using MoveBaseClient = actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction>;

/**
 * @brief Implementation of publisher being able of sending goals to move_base.
 */
class GoalPublisherMoveBase : public GoalPublisher {
public:
  GoalPublisherMoveBase() = default;
  virtual ~GoalPublisherMoveBase() = default;
  void sendGoal(geometry_msgs::TransformStamped &pose) override;

private:
  MoveBaseClient move_base_client_ = MoveBaseClient("move_base", true);
};
