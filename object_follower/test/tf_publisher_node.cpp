#include "ros/node_handle.h"
#include "ros/rate.h"
#include <cmath>
#include <geometry_msgs/TransformStamped.h>
#include <ros/ros.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/transform_broadcaster.h>

constexpr double NODE_RATE = 5.0;

/// @brief Test helper method for publishing goal tf
class tfPublisher {

public:
  tfPublisher(double x_min_val, double x_max_val, double angle_min_val, double angle_max_val)
      : nh_(), pnh_("~") {
    setAngleConstraints(angle_min_val, angle_max_val);
    setRangeConstraints(x_min_val, x_max_val);

    pnh_.param<bool>("is_x_forward", is_x_forward_, true);
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

    for (int i = 0, j = 0; j < step_count_; j++, i++) {
      pos_x_value = x_min_ + x_increment_ * i;
      angle_value = min_angle_ + angle_increment_ * j;

      /* ROS_INFO_THROTTLE(5, "Iteration %d. Pose x: %f, angle: %f", i, pos_x_value, angle_value); */
      current_position_.setX(pos_x_value);
      iterateBroadcating(current_position_, angle_value);
      rate_.sleep();
    }
  }

  void setRangeConstraints(double in_x_min, double in_x_max) {
    x_min_ = in_x_min;
    x_max_ = in_x_max;
    x_increment_ = (in_x_max - in_x_min) / step_count_;
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

    if (is_x_forward_) {
      q.setRPY(0, 0, theta);
    } else {
      q.setRPY(theta, M_PI / 2, 0);
    }

    transformStamped.transform.rotation.x = q.x();
    transformStamped.transform.rotation.y = q.y();
    transformStamped.transform.rotation.z = q.z();
    transformStamped.transform.rotation.w = q.w();

    br.sendTransform(transformStamped);
  }

protected:
  ros::NodeHandle nh_;
  ros::NodeHandle pnh_;

  std::string base_frame_ = "map";
  std::string object_frame_ = "object";
  tf2::Vector3 current_position_ = {0, 0.3, 0};

  ros::Rate rate_ = ros::Rate(NODE_RATE);

  bool is_x_forward_;
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

  double x_min = 1.0;
  double x_max = 3.0;

  double angle_min = -M_PI;
  double angle_max = M_PI;

  tfPublisher tfPub(x_min, x_max, angle_min, angle_max);
  tfPub.start();

  return 0;
};
