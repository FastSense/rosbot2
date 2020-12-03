#include "object_follower_tf_broadcaster_node.hpp"

constexpr double NODE_RATE = 10.0;

namespace Follower {

TfBroadcasterNode::TfBroadcasterNode() : node_rate_(NODE_RATE) {}

void TfBroadcasterNode::sleep() { node_rate_.sleep(); }

void TfBroadcasterNode::start() {
  while (ros::ok()) {
    try {
      follow();
    } catch (...) {
      exceptionFilter();
    }

    ros::spinOnce();
    sleep();
  }
}

} // namespace Follower

