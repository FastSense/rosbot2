#include "object_follower_core.hpp"

constexpr double SERVER_WAIT_DURATION = 5.0;

namespace Follower {

ObjectFollower::ObjectFollower() { tf_listener_ = std::make_unique<tfListener>(tf_buffer_); }

auto ObjectFollower::checkTf() const -> void {
  tf_buffer_.canTransform(base_frame_, object_frame_, tf_oldness_);
}

auto ObjectFollower::getTf() const -> tfStamped {
  tfStamped tf_pose;
  tf_pose = tf_buffer_.lookupTransform(base_frame_, object_frame_, tf_oldness_);
  return tf_pose;
}

}; // namespace Follower

