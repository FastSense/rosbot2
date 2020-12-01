#include "object_follower_tf_broadcaster.hpp"

namespace Follower {

TfBroadcaster::TfBroadcaster() {
  pnh_.param<std::string>("goal_frame_", goal_frame_, "goal_to_follow");
}

auto TfBroadcaster::follow() -> void {
  if (!enable_following_)
    return;

  try {
    auto pose_tf = getTf();
    setGoalTf(pose_tf);
    if (!updatePoseIfGood(pose_tf))
      return;
    broadcast(pose_tf);
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

auto TfBroadcaster::broadcast(tfStamped &pose) -> void {
  pose.child_frame_id = goal_frame_;
  tf_broadcaster_.sendTransform(pose);
}

}; // namespace Follower
