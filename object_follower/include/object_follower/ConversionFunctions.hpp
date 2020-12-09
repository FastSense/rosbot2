#pragma once

#include "geometry_msgs/TransformStamped.h"
#include "tf2/LinearMath/Matrix3x3.h"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2/LinearMath/Vector3.h"

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

class ConversionFunctions {
public:
  ConversionFunctions() = default;
  ~ConversionFunctions() = default;

  static PoseTf convertToTfPair(const geometry_msgs::TransformStamped &pose);
  static RPY getRPYfromQuaternion(const geometry_msgs::Quaternion &rotation);
};
