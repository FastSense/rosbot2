#include "object_follower_2d.hpp"
#include "geometry_msgs/Vector3Stamped.h"
#include "tf2/LinearMath/Quaternion.h"
#include <tuple>

namespace Follower {

ObjectFollower2d::ObjectFollower2d() {
  bool state = false;
  while (!state)
    try {
      current_position_ = getTf();
      setGoalTf(current_position_);
      state = true;
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

auto ObjectFollower2d::setGoalTf(tfStamped &pose) const -> void {
  double yaw = getYawFromQuaternion(pose.transform.rotation);
  Vector3 &translation = pose.transform.translation;

  setGoalTranslation(translation, yaw);
  setGoalRotation(pose, yaw);
}

auto ObjectFollower2d::getYawFromQuaternion(const Quaternion &q) const noexcept -> double {
  double siny_cosp = 2.0 * (q.w * q.z + q.x * q.y);
  double cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z);
  double yaw = std::atan2(siny_cosp, cosy_cosp);
  return yaw;
}

auto ObjectFollower2d::setGoalRotation(tfStamped &pose, const double yaw) const -> void {
  tf2::Quaternion q;
  q.setRPY(0, 0, M_PI + yaw);

  pose.transform.rotation.w = q.w();
  pose.transform.rotation.x = q.x();
  pose.transform.rotation.y = q.y();
  pose.transform.rotation.z = q.z();
}

auto ObjectFollower2d::setGoalTranslation(Vector3 &pt, const double yaw) const -> void {
  const double &r = goal_dist_from_obj_;
  pt.x = pt.x + r * cos(yaw);
  pt.y = pt.y + r * sin(yaw);
  pt.z = 0;
}

auto ObjectFollower2d::getTfPoseFromMsg(const tfStamped &pose) const -> PoseTf {
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

auto ObjectFollower2d::updatePoseIfGood(const tfStamped &pose) -> bool {
  if (isNewGoalGood(pose)) {
    ROS_INFO("New Pose accepted");
    setCurrentPosition(pose);
  } else {
    return false;
  }

  return true;
}

auto ObjectFollower2d::setCurrentPosition(const tfStamped &pose) -> void {
  current_position_ = pose;
}

auto ObjectFollower2d::isNewGoalGood(const tfStamped &pose) const -> bool {
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

