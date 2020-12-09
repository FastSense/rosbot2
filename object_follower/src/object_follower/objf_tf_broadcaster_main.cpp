#include "objf_tf_broadcaster_node.hpp"
#include "ros/init.h"

using namespace Follower;

int main(int argc, char **argv) {
  ros::init(argc, argv, "object_follower");
  TfBroadcasterFollowerNode node;
  ROS_INFO("Object Follower Start Working");
  node.start();
}
