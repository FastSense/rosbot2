#include "object_follower.hpp"
#include "ros/init.h"
#include <ros/ros.h>

constexpr double NODE_RATE = 10.0;

namespace Follower {

class ObjectFollowerNode : public ObjectFollower {
public:
  ObjectFollowerNode() : ObjectFollower(), nh_(), pnh_("~"), node_rate_(NODE_RATE) {
    pnh_.param<std::string>("base_frame", base_frame_, "map");
    pnh_.param<std::string>("object_frame_", object_frame_, "object");
    pnh_.param<std::string>("goal_frame_", goal_frame_, "goal_to_follow");

    pnh_.param<double>("range_diff_to_set_new_pose", range_diff_to_set_new_pose_, 1.0);
    pnh_.param<double>("yaw_diff_to_set_new_pose", yaw_diff_to_set_new_pose_, 20.0);
    pnh_.param<double>("max_dist_to_obj", max_dist_to_obj_, 5.0);
    pnh_.param<double>("goal_dist_from_obj", goal_dist_from_obj_, 1.0);

    pnh_.param<bool>("send_tf_instead", send_tf_instead_, false);
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
    node.follow();
    node.sleep();
  }
}
