#include "ros/rate.h"
#include <cmath>
#include <geometry_msgs/TransformStamped.h>
#include <ros/ros.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/transform_broadcaster.h>

constexpr double NODE_RATE = 10.0;

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

  double angle_min = -M_PI;
  double angle_max = M_PI;
  double angle_increment = M_PI / 180;
  double step_count = (angle_max - angle_min) / angle_increment;

  double x_min = 0;
  double x_max = 3;
  double x_step = (x_max - x_min) / step_count;

  ros::Rate rate = ros::Rate(NODE_RATE);

  ROS_INFO("Starting Broadcast");
  tf2::Vector3 pose = {0, 0, 0};

  while (ros::ok()) {
    int i = 0;
    for (double angle = angle_min; angle < angle_max; angle += angle_increment) {
      double curr_pose;
      curr_pose = x_min + x_step * (i++);

      ROS_INFO("Iteration %d. Pose x: %f, Broadcasting angle: %f", i, curr_pose, angle);
      pose.setX(curr_pose);
      broadcast(pose, angle);
      rate.sleep();
    }
  }

  return 0;
};
