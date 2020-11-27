#include "object_follower.hpp"
#include "ros/init.h"
#include <ros/ros.h>

constexpr double NODE_RATE = 10.0;

namespace Follower {

class ObjectFollowerNode : public ObjectFollower {
public:
  ObjectFollowerNode() : ObjectFollower(), nh_(), pnh_("~"), node_rate_(NODE_RATE) {
    // setParams
  }
  void sleep() { node_rate_.sleep(); }

private:
  ros::NodeHandle nh_;
  ros::NodeHandle pnh_;
  ros::Rate node_rate_;
};

} // namespace Follower

using namespace Follower;

int main(int argc, char **argv) {
  ros::init(argc, argv, "aruco_detect");
  ObjectFollowerNode node;

  ROS_INFO("Object Follower Start Working");

  while (ros::ok()) {
    node.sendTfToFollow();
    node.sleep();
  }
}
