#include "object_follower_core.hpp"

constexpr double NODE_RATE = 10.0;
const inline std::string SERVICE_NAME = "enable_following";

namespace Follower {

ObjectFollower::ObjectFollower() : nh_(), pnh_("~") {
  setParams();

  tf_listener_ = std::make_unique<tfListener>(tf_buffer_);
  tf_wait_ = ros::Duration(tf_wait_value_);

  service_enable_following =
      nh_.advertiseService(SERVICE_NAME, &ObjectFollower::enableFollowingCb, this);
}

auto ObjectFollower::setParams() -> void {
  pnh_.param<std::string>("base_frame", base_frame_, "map");
  pnh_.param<std::string>("object_frame_", object_frame_, "object");

  pnh_.param<double>("range_diff_to_set_new_pose", range_diff_to_set_new_pose_, 0.2);
  pnh_.param<double>("yaw_diff_to_set_new_pose", angle_diff_to_set_new_pose_, 40.0);

  pnh_.param<double>("tf_wait", tf_wait_value_, 1.0);
  pnh_.param<double>("goal_dist_from_obj", goal_dist_from_obj_, 1.0);
}

auto ObjectFollower::enableFollowingCb(Request &req, Response &res) -> bool {
  enable_following_ = req.data;
  ROS_INFO("Following set to %d", enable_following_);
  res.success = true;
  return true;
}

auto ObjectFollower::getTf() const -> tfStamped {
  static ros::Time tf_oldness_;
  tf_oldness_ = ros::Time(ros::Time::now());
  tfStamped tf_pose;
  tf_pose = tf_buffer_.lookupTransform(base_frame_, object_frame_, tf_oldness_, tf_wait_);
  return tf_pose;
}

auto ObjectFollower::getTfPoseFromMsg(const tfStamped &pose) const -> PoseTf {
  auto t = pose.transform.translation;
  auto q = pose.transform.rotation;

  Vector3Tf t_tf;
  QuaternionTf q_tf;

  t_tf.setValue(t.x, t.y, t.z);
  q_tf.setW(q.w);
  q_tf.setX(q.x);
  q_tf.setY(q.y);
  q_tf.setZ(q.z);

  return PoseTf{t_tf, q_tf};
}

auto ObjectFollower::updatePoseIfGood(const tfStamped &pose) -> bool {
  bool state = false;
  if ((state = isNewGoalGood(pose))) {
    ROS_INFO("New Pose accepted");
    setCurrentPosition(pose);
  }

  return state;
}

auto ObjectFollower::setCurrentPosition(const tfStamped &pose) -> void { current_position_ = pose; }

auto ObjectFollower::isNewGoalGood(const tfStamped &pose) const -> bool {
  auto [new_position, new_quaternion] = getTfPoseFromMsg(pose);
  auto [old_position, old_quaternion] = getTfPoseFromMsg(current_position_);

  double dist = tf2::tf2Distance(old_position, new_position);
  double angle_in_radian = tf2::angle(old_quaternion, new_quaternion);
  double angle_in_degrees = angle_in_radian * 180.0 / M_PI;

  const double &angle = angle_in_degrees;
  const double &max_dist = range_diff_to_set_new_pose_;
  const double &max_angle = angle_diff_to_set_new_pose_;

  return (dist > max_dist || angle > max_angle) ? true : false;
}

}; // namespace Follower

