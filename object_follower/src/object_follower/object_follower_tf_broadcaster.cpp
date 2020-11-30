#include "object_follower_tf_broadcaster.hpp"

namespace Follower {

tfBroadcaster::tfBroadcaster() {}

auto tfBroadcaster::follow() -> void {
  if (!enable_following_)
    return;

  try {
    checkTf();
    auto pose_tf = getTf();
    setGoalTf(pose_tf);
    broadcast(pose_tf);
  } catch (tf2::LookupException &ex) {
    ROS_WARN("Object frame not found: %s", ex.what());
  } catch (tf2::TimeoutException &ex) {
    ROS_WARN("Object frame lookup exceed it's time limit : %s", ex.what());
  } catch (tf2::ConnectivityException &ex) {
    ROS_WARN("Object frame not connected to base frame !: %s", ex.what());
  } catch (...) {
    ROS_ERROR("Error, can't send tf");
  }
}

auto tfBroadcaster::broadcast(const tfStamped &pose) -> void {
  tf_broadcaster_.sendTransform(pose);
}

}; // namespace Follower
