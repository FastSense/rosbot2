#pragma once

#include "GoalPublisher.hpp"
#include <tf2_ros/transform_broadcaster.h>
#include "geometry_msgs/PoseStamped.h"

class GoalPublisherTf : public GoalPublisher {
public:
  virtual ~GoalPublisherTf() = default;

protected:
  void sendGoal(geometry_msgs::TransformStamped &pose) override;

private:
  tf2_ros::TransformBroadcaster tf_broadcaster_;
  std::string goal_frame_ = "goal_tf_frame";
};
