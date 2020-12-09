#include "ConversionFunctions.hpp"

PoseTf ConversionFunctions::convertToTfPair(const geometry_msgs::TransformStamped &pose) {
  tf2::Vector3 t_tf;
  tf2::Quaternion q_tf;

  tf2::convert(pose.transform.translation, t_tf);
  tf2::convert(pose.transform.rotation, q_tf);

  return PoseTf{t_tf, q_tf};
}

RPY ConversionFunctions::getRPYfromQuaternion(const geometry_msgs::Quaternion &rotation) {
  tf2::Quaternion q_tf;
  tf2::convert(rotation, q_tf);
  double r, p, y;
  tf2::Matrix3x3(q_tf).getRPY(r, p, y);

  return RPY{r, p, y};
}
