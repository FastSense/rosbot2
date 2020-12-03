#include "ros/rate.h"
#include <cmath>
#include <geometry_msgs/TransformStamped.h>
#include <ros/ros.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/transform_broadcaster.h>

constexpr double NODE_RATE = 10.0;

/// @brief Test helper method for publishing goal tf
class tfPublisher {

public:
  tfPublisher(double x_min_val, double x_max_val, double angle_min_val, double angle_max_val) {
    setAngleConstraints(angle_min_val, angle_max_val);
    setRangeConstraints(x_min_val, x_max_val);
  }

  void start() {
    while (ros::ok()) {
      broadcast();
    }
  }

private:
  void broadcast() {
    double pos_x_value;
    double angle_value;

    ROS_INFO("STEP COUNT %f", step_count_);
    ROS_INFO("X_MIN %f", x_min_);
    ROS_INFO("X_INCREMENT %f", x_increment_);
    ROS_INFO("MIN_ANGLE %f", min_angle_);
    ROS_INFO("angle_increment_ %f", angle_increment_);
    for (int i = 0, j = 0; j < step_count_; j++, i++) {
      pos_x_value = x_min_ + x_increment_ * i;
      angle_value = min_angle_ + angle_increment_ * j;

      ROS_INFO("Iteration %d. Pose x: %f, angle: %f", i, pos_x_value, angle_value);
      current_position_.setX(pos_x_value);
      iterateBroadcating(current_position_, angle_value);
      rate.sleep();
    }
  }

  void setRangeConstraints(double in_x_min, double in_x_max) {
    x_min_ = in_x_min;
    x_max_ = in_x_max;
    x_increment_ = (in_x_max - in_x_min) / step_count_;
    ROS_INFO("SET XMIN =%f ; XMAX =%f ; XSTEP =%f, STEP_COUNT = %f, ANGLEINCREMENT = %f", x_min_,
             x_max_, x_increment_, step_count_, angle_increment_);
  }

  void setAngleConstraints(double in_min_angle, double in_max_angle) {
    max_angle_ = in_max_angle;
    min_angle_ = in_min_angle;
    angle_increment_ = M_PI / 180;
    step_count_ = abs((in_max_angle - in_min_angle)) / angle_increment_;
  }

  void iterateBroadcating(const tf2::Vector3 &pose, double theta) {
    static tf2_ros::TransformBroadcaster br;
    geometry_msgs::TransformStamped transformStamped;

    transformStamped.header.stamp = ros::Time::now();
    transformStamped.header.frame_id = base_frame_;
    transformStamped.child_frame_id = object_frame_;

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

protected:
  std::string base_frame_ = "map";
  std::string object_frame_ = "object";
  tf2::Vector3 current_position_ = {0, 0, 0};

  ros::Rate rate = ros::Rate(NODE_RATE);

  double min_angle_;
  double max_angle_;

  double x_min_;
  double x_max_;

  double step_count_;

  double angle_increment_;
  double x_increment_;
};

int main(int argc, char **argv) {
  ros::init(argc, argv, "tf_broacaster");
  ros::NodeHandle nh;

  double x_min = 1.0;
  double x_max = 3.0;

  double angle_min = -M_PI;
  double angle_max = M_PI;

  tfPublisher tfPub(x_min, x_max, angle_min, angle_max);
  tfPub.start();

  return 0;
};
