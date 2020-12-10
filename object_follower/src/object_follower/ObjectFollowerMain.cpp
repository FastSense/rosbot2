#include "ObjectFollowerFarm.hpp"
#include "ros/init.h"
#include "ros/node_handle.h"

int main(int argc, char **argv) {
  ros::init(argc, argv, "object_follower");
  ObjectFollowerFarm farm;
  farm.setParams();
  auto node = farm.makeFollower();

  ROS_INFO("Object Follower Start Working");
  node.start();
}
