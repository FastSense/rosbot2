#pragma once

#include "geometry_msgs/TransformStamped.h"
#include "ros/rate.h"

constexpr float rate_value = 10;

class GoalPublisher {
public:
  GoalPublisher() : rate_(rate_value){};
  virtual ~GoalPublisher() = default;
  virtual void sendGoal(geometry_msgs::TransformStamped &pose) = 0;

public:
  std::string base_frame_;
  ros::Rate rate_;
};
