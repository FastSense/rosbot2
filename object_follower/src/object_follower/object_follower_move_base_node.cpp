#include "object_follower_move_base_node.hpp"

constexpr double NODE_RATE = 10.0;

namespace Follower {

MoveBaseFollowerNode::MoveBaseFollowerNode() : node_rate_(NODE_RATE) {}

auto MoveBaseFollowerNode::sleep() -> void { node_rate_.sleep(); }

auto MoveBaseFollowerNode::start() -> void {
  while (ros::ok()) {
    follow();
    ros::spinOnce();
    sleep();
  }
}

} // namespace Follower

