#include "object_follower_move_base_node.hpp"
#include "ros/init.h"

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

using namespace Follower;

int main(int argc, char **argv) {
  ros::init(argc, argv, "object_follower");
  MoveBaseFollowerNode node;
  ROS_INFO("Object Follower Start Working");
  node.start();
}
