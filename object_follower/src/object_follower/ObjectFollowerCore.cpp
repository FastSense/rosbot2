#include "ObjectFollowerCore.hpp"

#include "GoalChecker.hpp"
#include "GoalGenerator.hpp"
#include "GoalPublisher.hpp"

#include <memory>

const inline std::string SERVICE_NAME = "enable_following";
constexpr double LOGGER_MESSEGE_PERIOD = 20;

ObjectFollowerCore::ObjectFollowerCore(std::unique_ptr<GoalChecker> goal_checker,
                                       std::unique_ptr<GoalGenerator> goal_generator,
                                       std::unique_ptr<GoalPublisher> goal_publisher)
    : goal_checker_(std::move(goal_checker)), goal_generator_(std::move(goal_generator)),
      goal_publisher_(std::move(goal_publisher)), nh_(), pnh_("~") {

  service_enable_following_ =
      pnh_.advertiseService(SERVICE_NAME, &ObjectFollowerCore::enableFollowingCb, this);

  setParams();
}

void ObjectFollowerCore::start() {
  while (ros::ok()) {
    try {
      follow();
    } catch (...) {
      exceptionFilter();
    }

    ros::spinOnce();
    sleep();
  }
}

void ObjectFollowerCore::follow() {
  if (not following_enabled_)
    return;

  geometry_msgs::TransformStamped pose;

  pose = goal_generator_->lookupTransform();
  goal_generator_->evalGoal(pose);
  if (goal_checker_->isGoalConsiderable(pose)) {
    goal_publisher_->sendGoal(pose);
    goal_checker_->setCurrentPosition(pose);
  }
}

void ObjectFollowerCore::sleep() const { goal_publisher_->rate_.sleep(); }

bool ObjectFollowerCore::enableFollowingCb(std_srvs::SetBool::Request &req,
                                           std_srvs::SetBool::Response &res) {
  following_enabled_ = req.data;
  ROS_INFO("Following set to %d", following_enabled_);
  res.success = true;
  return true;
}

void ObjectFollowerCore::setParams() {
  pnh_.param<std::string>("base_frame", goal_generator_->base_frame_, "map");
  pnh_.param<std::string>("object_frame", goal_generator_->object_frame_, "object");
  pnh_.param<double>("goal_dist_from_obj", goal_generator_->goal_dist_from_obj_, 0.5);

  pnh_.param<double>("range_diff_to_set_new_pose", goal_checker_->range_diff_to_set_new_pose_,
                     0.15);
  pnh_.param<double>("angle_diff_to_set_new_pose", goal_checker_->angle_diff_to_set_new_pose_,
                     10.0);
}

void ObjectFollowerCore::exceptionFilter() {
  try {
    throw;
  } catch (tf2::LookupException &ex) {
    ROS_WARN_THROTTLE(LOGGER_MESSEGE_PERIOD, "Object frame not found: %s", ex.what());
  } catch (tf2::TimeoutException &ex) {
    ROS_WARN_THROTTLE(LOGGER_MESSEGE_PERIOD, "Object frame lookup exceed it's time limit : %s",
                      ex.what());
  } catch (tf2::ConnectivityException &ex) {
    ROS_WARN_THROTTLE(LOGGER_MESSEGE_PERIOD, "Object frame not connected to base frame !: %s",
                      ex.what());
  } catch (tf2::ExtrapolationException &ex) {
    ROS_INFO_THROTTLE(LOGGER_MESSEGE_PERIOD, "Latest tf is too old: %s", ex.what());
  } catch (ros::Exception &ex) {
    ROS_WARN_THROTTLE(LOGGER_MESSEGE_PERIOD, "ROS exception caught: %s", ex.what());
  } catch (...) {
    ROS_ERROR_THROTTLE(LOGGER_MESSEGE_PERIOD, "Unpredictable error, can't send goal");
  }
}

