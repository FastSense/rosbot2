#pragma once

#include "geometry_msgs/TransformStamped.h"
#include "ros/rate.h"

constexpr float RATE_VALUE = 10.0;

class GoalPublisher {
public:
  GoalPublisher() : rate_(RATE_VALUE){};
  virtual ~GoalPublisher() = default;
  virtual void sendGoal(geometry_msgs::TransformStamped &pose) = 0;

public:
  std::string base_frame_;
  ros::Rate rate_;
};
