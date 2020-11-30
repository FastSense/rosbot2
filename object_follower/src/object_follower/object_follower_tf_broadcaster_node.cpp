#include "object_follower_tf_broadcaster_node.hpp"
#include "ros/init.h"

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

using namespace Follower;

int main(int argc, char **argv) {
  ros::init(argc, argv, "object_follower");
  TfBroadcasterNode node;
  ROS_INFO("Object Follower Start Working");
  node.start();
}
