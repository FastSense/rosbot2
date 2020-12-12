#pragma once

#include "geometry_msgs/TransformStamped.h"
#include <memory>
#include <ros/ros.h>
#include <ros/service_server.h>
#include <std_srvs/SetBool.h>

#include "object_follower/FollowerParamsConfig.h"
#include <dynamic_reconfigure/server.h>

class GoalChecker;
class GoalGenerator;
class GoalPublisher;

using DynamicServer = dynamic_reconfigure::Server<object_follower::FollowerParamsConfig>;
using DynamicCallback =
    dynamic_reconfigure::Server<object_follower::FollowerParamsConfig>::CallbackType;
/**
 * @brief Class unites GoalPublisher GoalChecker and GoalGenerator components and provides
 * functional for combined work of them and following to object
 */
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

  void dynamicConfigCb(object_follower::FollowerParamsConfig &config, uint32_t level);

private:
  ros::NodeHandle nh_;
  ros::NodeHandle pnh_;

  bool following_enabled_ = true;
  ros::ServiceServer service_enable_following_;

  std::unique_ptr<DynamicServer> config_server_;
  DynamicCallback config_callback_;

  std::unique_ptr<GoalChecker> goal_checker_;
  std::unique_ptr<GoalGenerator> goal_generator_;
  std::unique_ptr<GoalPublisher> goal_publisher_;
};

