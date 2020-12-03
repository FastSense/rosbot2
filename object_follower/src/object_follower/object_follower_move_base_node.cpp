#include "object_follower_move_base_node.hpp"

constexpr double NODE_RATE = 10.0;

namespace Follower {

MoveBaseFollowerNode::MoveBaseFollowerNode() : node_rate_(NODE_RATE) {}

void MoveBaseFollowerNode::sleep() { node_rate_.sleep(); }

void MoveBaseFollowerNode::start() {
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

