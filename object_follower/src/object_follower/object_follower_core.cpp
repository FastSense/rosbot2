#include "object_follower_core.hpp"
#include <memory>

constexpr double NODE_RATE = 10.0;
static const inline std::string SERVICE_NAME = "enable_following";

namespace Follower {

ObjectFollower::ObjectFollower() : nh_(), pnh_("~") {
  tf_listener_ = std::make_unique<tfListener>(tf_buffer_);
  service_enable_following =
      nh_.advertiseService(SERVICE_NAME, &ObjectFollower::enableFollowingCb, this);

  setParams();
}

auto ObjectFollower::setParams() -> void {
  pnh_.param<std::string>("base_frame", base_frame_, "map");
  pnh_.param<std::string>("object_frame_", object_frame_, "object");

  pnh_.param<double>("range_diff_to_set_new_pose", range_diff_to_set_new_pose_, 1.0);
  pnh_.param<double>("yaw_diff_to_set_new_pose", angle_diff_to_set_new_pose_, 20.0);
  pnh_.param<double>("max_dist_to_obj", max_dist_to_obj_, 5.0);

  pnh_.param<double>("goal_dist_from_obj", goal_dist_from_obj_, 1.0);
}

auto ObjectFollower::enableFollowingCb(Request &req, Response &res) -> bool {
  enable_following_ = req.data;
  ROS_INFO("Following set to %d", enable_following_);
  res.success = true;
  return true;
}

auto ObjectFollower::checkTf() const -> void {
  static ros::Time tf_oldness_;
  tf_oldness_ = ros::Time(ros::Time::now());

  tf_buffer_.canTransform(base_frame_, object_frame_, tf_oldness_, tf_wait_);
}

auto ObjectFollower::getTf() const -> tfStamped {
  static ros::Time tf_oldness_;
  tf_oldness_ = ros::Time(ros::Time::now());
  tfStamped tf_pose;
  tf_pose = tf_buffer_.lookupTransform(base_frame_, object_frame_, tf_oldness_, tf_wait_);
  return tf_pose;
}

}; // namespace Follower

