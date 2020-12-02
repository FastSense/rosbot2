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

struct PoseTf {
  Vector3Tf translation;
  QuaternionTf quaternion;
};

class ObjectFollower {
public:
  ObjectFollower();
  virtual void start() = 0;
  ~ObjectFollower() = default;

protected:
  virtual void follow() = 0;
  virtual void sleep() = 0;

  tfStamped getTf() const;
  bool enableFollowingCb(Request &req, Response &res);

  bool updatePoseIfGood(const tfStamped &pose);
  bool isNewGoalGood(const tfStamped &pose) const;
  PoseTf getTfPoseFromMsg(const tfStamped &pose) const;
  void setCurrentPosition(const tfStamped &pose);

  void exceptionFilter() const;

  std::optional<tfStamped> current_position_;

private:
  void setParams();

protected:
  std::string base_frame_;
  std::string object_frame_;

  bool following_enabled_ = true;
  double goal_dist_from_obj_;

  double range_diff_to_set_new_pose_;
  double angle_diff_to_set_new_pose_;

  double tf_wait_value_;
  ros::Duration tf_wait_;
  ros::NodeHandle nh_;
  ros::NodeHandle pnh_;

private:
  std::unique_ptr<tfListener> tf_listener_;
  tf2_ros::Buffer tf_buffer_;
  ros::ServiceServer service_enable_following;
};

}; // namespace Follower
