#include "ros/rate.h"
#include <cmath>
#include <geometry_msgs/TransformStamped.h>
#include <ros/ros.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/transform_broadcaster.h>

constexpr double NODE_RATE = 50.0;

void broadcast(const tf2::Vector3 &pose, double theta) {

  static tf2_ros::TransformBroadcaster br;

  geometry_msgs::TransformStamped transformStamped;
  transformStamped.header.stamp = ros::Time::now();
  transformStamped.header.frame_id = "map";
  transformStamped.child_frame_id = "object";

  transformStamped.transform.translation.x = pose.x();
  transformStamped.transform.translation.y = pose.y();
  transformStamped.transform.translation.z = pose.z();
  tf2::Quaternion q;

  q.setRPY(0, 0, theta);
  transformStamped.transform.rotation.x = q.x();
  transformStamped.transform.rotation.y = q.y();
  transformStamped.transform.rotation.z = q.z();
  transformStamped.transform.rotation.w = q.w();

  br.sendTransform(transformStamped);
}

int main(int argc, char **argv) {
  ros::init(argc, argv, "tf_broacaster");
  ros::NodeHandle nh;

  tf2::Vector3 pose = {1, 1, 0};
  double angle_min = -M_PI;
  double angle_max = M_PI ;
  double angle_increment = M_PI / 180;

  ros::Rate rate = ros::Rate(NODE_RATE);

  ROS_INFO("Starting Broadcast");
  while (ros::ok()) {
    for (double angle = angle_min; angle < angle_max; angle += angle_increment) {
      ROS_INFO("Broadcasting angle: %f", angle);
      broadcast(pose, angle);
      rate.sleep();
    }
  }

  return 0;
};
