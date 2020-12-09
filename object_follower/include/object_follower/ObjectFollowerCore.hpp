#pragma once

#include "geometry_msgs/TransformStamped.h"
#include <memory>
#include <ros/ros.h>
#include <ros/service_server.h>
#include <std_srvs/SetBool.h>

#include "GoalChecker.hpp"
#include "GoalGenerator.hpp"
#include "GoalPublisher.hpp"

class ObjectFollowerCore {
public:
  ObjectFollowerCore() = default;
  virtual ~ObjectFollowerCore() = default;

  ObjectFollowerCore(std::unique_ptr<GoalChecker> goal_checker,
                     std::unique_ptr<GoalGenerator> goal_generator,
                     std::unique_ptr<GoalPublisher> goal_publisher);

  void start();

protected:
  void sleep();

  bool enableFollowingCb(std_srvs::SetBool::Request &req, std_srvs::SetBool::Response &res);
  void setParams();
  void exceptionFilter() const;

protected:
  ros::NodeHandle nh_;
  ros::NodeHandle pnh_;

  bool following_enabled_;
  ros::ServiceServer service_enable_following_;

private:
  std::unique_ptr<GoalChecker> goal_checker_;
  std::unique_ptr<GoalGenerator> goal_generator_;
  std::unique_ptr<GoalPublisher> goal_publisher_;
};

