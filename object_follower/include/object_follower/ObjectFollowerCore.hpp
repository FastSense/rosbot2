#pragma once

#include <memory>
#include <ros/ros.h>
#include <ros/service_server.h>
#include <std_srvs/SetBool.h>

class GoalChecker;
class GoalGenerator;
class GoalPublisher;

class ObjectFollowerCore {
public:
  ObjectFollowerCore() = delete;
  virtual ~ObjectFollowerCore() = default;
  ObjectFollowerCore(ObjectFollowerCore &&obj) = default;

  ObjectFollowerCore(std::unique_ptr<GoalChecker> goal_checker,
                     std::unique_ptr<GoalGenerator> goal_generator,
                     std::unique_ptr<GoalPublisher> goal_publisher);

  void start();

private:
  void follow();
  void sleep() const;

  bool enableFollowingCb(std_srvs::SetBool::Request &req, std_srvs::SetBool::Response &res);
  void setParams();
  static void exceptionFilter();

private:
  ros::NodeHandle nh_;
  ros::NodeHandle pnh_;

  bool following_enabled_ = true;
  ros::ServiceServer service_enable_following_;

  std::unique_ptr<GoalChecker> goal_checker_;
  std::unique_ptr<GoalGenerator> goal_generator_;
  std::unique_ptr<GoalPublisher> goal_publisher_;
};

