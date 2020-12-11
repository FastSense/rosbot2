#include "ObjectFollowerFactory.hpp"
#include "ros/init.h"

int main(int argc, char **argv) {
  ros::init(argc, argv, "object_follower");
  ObjectFollowerFactory factory;

  factory.setParams();
  auto node = factory.makeFollower();

  if (not node.has_value()) {
    ROS_ERROR("Shutting down Object Follower Node");
    ros::shutdown();
    return 1;
  }

  ROS_INFO("Object Follower Start Working");
  node->start();
}
