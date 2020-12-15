#include "Converter.hpp"
#include "tf2/convert.h"

PoseTf Converter::convertToTfPair(const geometry_msgs::TransformStamped &pose) {
  tf2::Vector3 v_tf;
  tf2::Quaternion q_tf;

  tf2::convert(pose.transform.translation, v_tf);
  tf2::convert(pose.transform.rotation, q_tf);

  return PoseTf{v_tf, q_tf};
}

RPY Converter::getRPYfromQuaternion(const geometry_msgs::Quaternion &rotation) {
  tf2::Quaternion q_tf;
  tf2::convert(rotation, q_tf);
  double r, p, y;
  tf2::Matrix3x3(q_tf).getRPY(r, p, y);

  return RPY{r, p, y};
}

void Converter::rotateQuaternion(geometry_msgs::Quaternion &q, const RPY &rot) {
  tf2::Quaternion q_tf, q_rot;
  tf2::convert(q, q_tf);

  q_rot.setRPY(rot.roll, rot.pitch, rot.yaw);
  q_tf = q_rot * q_tf;

  tf2::convert(q_tf, q);
}

void Converter::setQuaternionRotation(geometry_msgs::Quaternion &q, const RPY &rot) {
  tf2::Quaternion q_tf;
  tf2::convert(q, q_tf);
  q_tf.setRPY(rot.roll, rot.pitch, rot.yaw);
  tf2::convert(q_tf, q);
}

move_base_msgs::MoveBaseGoal
Converter::tfToMoveBaseGoal(const geometry_msgs::TransformStamped &pose) {
  move_base_msgs::MoveBaseGoal converted_pose;

  converted_pose.target_pose.pose.orientation.w = pose.transform.rotation.w;
  converted_pose.target_pose.pose.orientation.x = pose.transform.rotation.x;
  converted_pose.target_pose.pose.orientation.y = pose.transform.rotation.y;
  converted_pose.target_pose.pose.orientation.z = pose.transform.rotation.z;

  converted_pose.target_pose.pose.position.x = pose.transform.translation.x;
  converted_pose.target_pose.pose.position.y = pose.transform.translation.y;
  converted_pose.target_pose.pose.position.z = pose.transform.translation.z;

  converted_pose.target_pose.header.frame_id = pose.header.frame_id;
  converted_pose.target_pose.header.seq = pose.header.seq;
  converted_pose.target_pose.header.stamp = pose.header.stamp;

  return converted_pose;
}

tf2::Stamped<tf2::Transform>
Converter::convertToTfPose(const geometry_msgs::TransformStamped &pose) {
  tf2::Stamped<tf2::Transform> pose_tf;
  tf2::convert(pose, pose_tf);
  return pose_tf;
}
