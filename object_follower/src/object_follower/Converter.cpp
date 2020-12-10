#include "Converter.hpp"
#include "tf2/convert.h"

PoseTf Converter::convertToTfPair(const geometry_msgs::TransformStamped &pose) {
  tf2::Vector3 t_tf;
  tf2::Quaternion q_tf;

  tf2::convert(pose.transform.translation, t_tf);
  tf2::convert(pose.transform.rotation, q_tf);

  return PoseTf{t_tf, q_tf};
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
  tf2::convert(q , q_tf);

  q_rot.setRPY(rot.roll, rot.pitch, rot.yaw);
  q_tf = q_rot * q_tf;

  tf2::convert(q_tf, q);
}

void Converter::setQuaternionRotation(geometry_msgs::Quaternion &q, const RPY &rot) {
  tf2::Quaternion q_tf;
  tf2::convert(q, q_tf);
  q_tf.setRPY(rot.roll,rot.pitch, rot.yaw);
  tf2::convert(q_tf, q);

}
