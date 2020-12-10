#pragma once

#include "GoalPublisher.hpp"
#include <tf2_ros/transform_broadcaster.h>
#include "geometry_msgs/PoseStamped.h"

class GoalPublisherTf : public GoalPublisher {
public:
  virtual ~GoalPublisherTf() = default;
  void sendGoal(geometry_msgs::TransformStamped &pose) override;
  std::string goal_frame_ = "goal_tf_frame";

private:
  tf2_ros::TransformBroadcaster tf_broadcaster_;
};
