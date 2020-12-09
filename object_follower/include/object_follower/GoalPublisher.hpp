#pragma once

#include "geometry_msgs/TransformStamped.h"
#include "ros/rate.h"

constexpr float rate_value = 10;

class GoalPublisher {
public:
  GoalPublisher();
  virtual ~GoalPublisher() = default;

  virtual void sendGoal(geometry_msgs::TransformStamped &pose) = 0;

  std::string base_frame_;
  ros::Rate rate_;
};
