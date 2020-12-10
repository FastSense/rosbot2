#pragma once

#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

struct RPY {
  double roll;
  double pitch;
  double yaw;
};

struct PoseTf {
  tf2::Vector3 translation;
  tf2::Quaternion quaternion;
};

class Converter {
public:
  Converter() = default;
  ~Converter() = default;

  static PoseTf convertToTfPair(const geometry_msgs::TransformStamped &pose);
  static RPY getRPYfromQuaternion(const geometry_msgs::Quaternion &rotation);
  static void rotateQuaternion(geometry_msgs::Quaternion &q, const RPY &rot); 
  static void setQuaternionRotation(geometry_msgs::Quaternion &q, const RPY &rot); 
};
