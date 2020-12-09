#include "objf_tf_broadcaster_node.hpp"

constexpr double NODE_RATE = 10.0;

namespace Follower {

TfBroadcasterFollowerNode::TfBroadcasterFollowerNode() : node_rate_(NODE_RATE) {}

void TfBroadcasterFollowerNode::sleep() { node_rate_.sleep(); }

void TfBroadcasterFollowerNode::start() {
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

