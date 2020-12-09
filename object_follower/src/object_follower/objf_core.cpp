#include "objf_core.hpp"

constexpr double NODE_RATE = 10.0;
const inline std::string SERVICE_NAME = "enable_following";

namespace Follower {

ObjectFollowerCore::ObjectFollowerCore() : nh_(), pnh_("~") {
  setParams();

  tf_listener_ = std::make_unique<tfListener>(tf_buffer_);
  tf_wait_ = ros::Duration(tf_wait_value_);

  service_enable_following_ =
      pnh_.advertiseService(SERVICE_NAME, &ObjectFollowerCore::enableFollowingCb, this);

  setGoalBaseFrame();
  broadcastNewOrigin();
}

void ObjectFollowerCore::setParams() {
  pnh_.param<std::string>("base_frame", base_frame_, "map");
  pnh_.param<std::string>("object_frame", object_frame_, "object");

  pnh_.param<double>("range_diff_to_set_new_pose", range_diff_to_set_new_pose_, 0.2);
  pnh_.param<double>("yaw_diff_to_set_new_pose", angle_diff_to_set_new_pose_, 15.0);

  pnh_.param<double>("tf_wait", tf_wait_value_, 1.0);
  pnh_.param<double>("goal_dist_from_obj", goal_dist_from_obj_, 1.0);

  int origin_type_;
  pnh_.param<int>("origin_type", origin_type_, 0);
  object_origin_ = static_cast<OriginType>(origin_type_);
}

void ObjectFollowerCore::setGoalBaseFrame() {
  goal_base_frame_ = base_frame_ + base_frame_postfix_;
}

void ObjectFollowerCore::broadcastNewOrigin() {

  tfStamped new_origin;

  new_origin.header.stamp = ros::Time::now();

  new_origin.header.frame_id = base_frame_;

  new_origin.child_frame_id = goal_base_frame_;

  new_origin.transform.translation.x = 0;
  new_origin.transform.translation.y = 0;
  new_origin.transform.translation.z = 0;
  tf2::Quaternion q;

  q.setRPY(0, 0, 0);

  ROS_INFO("%d", object_origin_);
  new_origin.transform.rotation.x = q.x();
  new_origin.transform.rotation.y = q.y();
  new_origin.transform.rotation.z = q.z();
  new_origin.transform.rotation.w = q.w();
  static_broadcaster_.sendTransform(new_origin);
}

bool ObjectFollowerCore::enableFollowingCb(Request &req, Response &res) {
  following_enabled_ = req.data;
  ROS_INFO("Following set to %d", following_enabled_);
  res.success = true;
  return true;
}

tfStamped ObjectFollowerCore::getTf() const {
  static ros::Time tf_oldness;

  tf_oldness = ros::Time(ros::Time::now());
  tfStamped tf_pose;
  tf_pose = tf_buffer_.lookupTransform(goal_base_frame_, object_frame_, tf_oldness, tf_wait_);
  return tf_pose;
}

PoseTf ObjectFollowerCore::convertPoseMsgToTf(const tfStamped &pose) const {
  Vector3Tf t_tf;
  QuaternionTf q_tf;

  tf2::convert(pose.transform.translation, t_tf);
  tf2::convert(pose.transform.rotation, q_tf);

  return PoseTf{t_tf, q_tf};
}

bool ObjectFollowerCore::updatePoseIfConsiderable(const tfStamped &pose) {
  if (!current_position_.has_value()) {
    current_position_ = pose;
    return true;
  }

  bool state = false;
  if ((state = isGoalConsiderable(pose))) {
    ROS_INFO("New Pose accepted");
    setCurrentPosition(pose);
  }

  return state;
}

void ObjectFollowerCore::setCurrentPosition(const tfStamped &pose) { current_position_ = pose; }

bool ObjectFollowerCore::isGoalConsiderable(const tfStamped &pose) const {
  auto [new_position, new_quaternion] = convertPoseMsgToTf(pose);
  auto [old_position, old_quaternion] = convertPoseMsgToTf(current_position_.value());

  double dist = tf2::tf2Distance(old_position, new_position);
  double angle_in_radian = tf2::angle(old_quaternion, new_quaternion);
  double angle_in_degrees = angle_in_radian * 180.0 / M_PI;

  bool isDistConsidirable = dist > range_diff_to_set_new_pose_;
  bool isAngleConsidirable = angle_in_radian > angle_diff_to_set_new_pose_;

  return (isDistConsidirable || isAngleConsidirable) ? true : false;
}

void ObjectFollowerCore::exceptionFilter() const {
  try {
    throw;
  } catch (tf2::LookupException &ex) {
    ROS_WARN("Object frame not found: %s", ex.what());
  } catch (tf2::TimeoutException &ex) {
    ROS_WARN("Object frame lookup exceed it's time limit : %s", ex.what());
  } catch (tf2::ConnectivityException &ex) {
    ROS_WARN("Object frame not connected to base frame !: %s", ex.what());
  } catch (tf2::ExtrapolationException &ex) {
    ROS_WARN("Extrapolation error : %s", ex.what());
  } catch (ros::Exception &ex) {
    ROS_WARN("ROS exception caught: %s", ex.what());
  } catch (...) {
    ROS_ERROR("Unpredictable error, can't send goal");
  }
}

}; // namespace Follower

