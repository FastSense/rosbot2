#pragma once
#include <move_base_msgs/MoveBaseGoal.h>
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

/**
 * @brief Class consists convert fucntions with some rotational & translational functional modules
 * on basic ros types.
 */
class Converter {
public:
  Converter() = default;
  ~Converter() = default;

  static PoseTf convertToTfPair(const geometry_msgs::TransformStamped &pose);
  static RPY getRPYfromQuaternion(const geometry_msgs::Quaternion &rotation);
  static move_base_msgs::MoveBaseGoal tfToMoveBaseGoal(const geometry_msgs::TransformStamped &pose);

  static void rotateQuaternion(geometry_msgs::Quaternion &q, const RPY &rot);
  static void setQuaternionRotation(geometry_msgs::Quaternion &q, const RPY &rot);
  static tf2::Stamped<tf2::Transform> convertToTfPose(const geometry_msgs::TransformStamped &pose);
};

