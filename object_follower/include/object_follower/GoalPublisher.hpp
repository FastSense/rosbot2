#pragma once

#include "geometry_msgs/TransformStamped.h"
#include "ros/rate.h"

constexpr float RATE_VALUE = 20.0;

/**
 * @brief Abstract class represents publishing goal algorithm.
 */
class GoalPublisher {
public:
  GoalPublisher() : rate_(RATE_VALUE){};
  virtual ~GoalPublisher() = default;
  virtual void sendGoal(geometry_msgs::TransformStamped &pose) = 0;

public:
  ros::Rate rate_;
};
