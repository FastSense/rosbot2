#include "object_follower_tf_broadcaster_node.hpp"

constexpr double NODE_RATE = 10.0;

namespace Follower {

TfBroadcasterNode::TfBroadcasterNode() : node_rate_(NODE_RATE) {}

auto TfBroadcasterNode::sleep() -> void { node_rate_.sleep(); }

auto TfBroadcasterNode::start() -> void {
  while (ros::ok()) {
    follow();
    ros::spinOnce();
    sleep();
  }
}

} // namespace Follower

