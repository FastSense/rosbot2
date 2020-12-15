#include "GoalGeneratorNearest2D.hpp"
#include "Converter.hpp"

#include <cmath>

geometry_msgs::TransformStamped GoalGeneratorNearest2D::evalGoal() {
  auto pose = lookupTransform(base_frame_, object_frame_);

  const double &r = goal_dist_from_obj_;
  geometry_msgs::Vector3 &translation = pose.transform.translation;
  geometry_msgs::Quaternion &rotation = pose.transform.rotation;

  double alpha = atan2(translation.y, translation.x);

  translation.x = translation.x - r * cos(alpha);
  translation.y = translation.y - r * sin(alpha);
  translation.z = 0;
  Converter::setQuaternionRotation(rotation, RPY{0, 0, alpha});

  auto map_base = lookupTransform(map_frame_, base_frame_);
  auto map_base_tf = Converter::convertToTfPose(map_base);
  auto base_object_tf = Converter::convertToTfPose(pose);
  auto &map_object_tf = map_base_tf;
  map_object_tf *= base_object_tf;

  tf2::convert(map_base_tf, pose);

  pose.header.frame_id = map_frame_;
  pose.child_frame_id = object_frame_;

  return pose;
}

