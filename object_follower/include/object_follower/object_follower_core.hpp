#pragma once

#include "ros/time.h"
#include "tf2_ros/transform_listener.h"
#include <optional>
#include <ros/ros.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

#include <ros/service_server.h>
#include <std_srvs/SetBool.h>

namespace Follower {

using tfStamped = geometry_msgs::TransformStamped;
using tfListener = tf2_ros::TransformListener;

using Request = std_srvs::SetBool::Request;
using Response = std_srvs::SetBool::Response;

using QuaternionTf = tf2::Quaternion;
using Vector3Tf = tf2::Vector3;

/// Implementation of simple pair of ObjectFollower::Vector3Tf and ObjectFollower::QuaternionTf
struct PoseTf {
  Vector3Tf translation;
  QuaternionTf quaternion;
};

/**
 * @brief Abstract Class consisting of basic functions to getting Tf cheking them as a goal,
 * NodeHandle's, frame names etc.
 */
class ObjectFollower {
public:
  ObjectFollower();

  /**
   * Abstract method should be implemented as "while(ros::ok())" cycle
   */
  virtual void start() = 0;

  ~ObjectFollower() = default;

protected:
  /**
   * Abstract method representing one cycle of following algorithm
   */
  virtual void follow() = 0;

  /**
   * Abstract method should invoke ros::Rate sleep in overload
   */
  virtual void sleep() = 0;

  /**
   * @throws tfLookupTransform related ros exceptions
   * @return returns tf from ObjectFollower::base_frame_ to ObjectFollower::object_frame_
   */
  tfStamped getTf() const;

  /**
   * Service callback to change ObjectFollower::following_enabled_
   */
  bool enableFollowingCb(Request &req, Response &res);

  /**
   * Invokes ObjectFollower::isNewGoalGood and if good sets ObjectFollower::current_position_
   * through ObjectFollower::setCurrentPosition
   */
  bool updatePoseIfGood(const tfStamped &pose);

  /**
   * Invokes ObjectFollower::isNewGoalGood and if good sets ObjectFollower::current_position_
   * @param[in] pose Pose to check
   * through ObjectFollower::setCurrentPosition
   */
  bool isNewGoalGood(const tfStamped &pose) const;

  /**
   * Retrieving ObjectFollower::PoseTf from ObjectFollower::tfStamped
   * @param[in] pose Pose to convert
   * @return PoseTf
   */
  PoseTf getTfPoseFromMsg(const tfStamped &pose) const;

  /**
   * Sets ObjectFollower::current_position_
   */
  void setCurrentPosition(const tfStamped &pose);

  /**
   * Exception filter for:
   * tf2::LookupException
   * tf2::TimeoutException
   * tf2::ConnectivityException
   * tf2::ExtrapolationException
   * ros::Exception
   * ... (all others)
   */
  void exceptionFilter() const;

  std::optional<tfStamped> current_position_;

private:
  void setParams(); /// sets params from parameter server

protected:
  std::string base_frame_;
  std::string object_frame_;

  bool following_enabled_ = true;

  /// distance from the object at which the goal will be set
  double goal_dist_from_obj_;

  /// new goal will be set only if farther then this (in meter)
  double range_diff_to_set_new_pose_;

  /**
   * new goal will be set only if new angle turned more then
   * this (in degree)
   */
  double angle_diff_to_set_new_pose_;

  double tf_wait_value_;
  ros::Duration tf_wait_;
  ros::NodeHandle nh_;
  ros::NodeHandle pnh_;

  std::unique_ptr<tfListener> tf_listener_;
  tf2_ros::Buffer tf_buffer_;

private:
  ros::ServiceServer service_enable_following;
};

}; // namespace Follower
