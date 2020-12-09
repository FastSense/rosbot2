#include "objf_move_base.hpp"
#include "ros/init.h"

using namespace Follower;

int main(int argc, char **argv) {
  ros::init(argc, argv, "object_follower");
  MoveBaseFollower node;
  ROS_INFO("Object Follower Start Working");
  node.start();
}
