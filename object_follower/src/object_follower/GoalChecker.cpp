#include "GoalChecker.hpp"
#include "Converter.hpp"

bool GoalChecker::isGoalConsiderable(const geometry_msgs::TransformStamped &pose) const {
  if (not current_position_.has_value()) {
    return true;
  }

  auto [new_position, new_quaternion] = Converter::convertToTfPair(pose);
  auto [old_position, old_quaternion] = Converter::convertToTfPair(current_position_.value());

  double dist = tf2::tf2Distance(old_position, new_position);
  double angle_in_radian = tf2::angle(old_quaternion, new_quaternion);
  double angle_in_degrees = angle_in_radian * 180.0 / M_PI;

  bool isDistConsidirable = dist > range_diff_to_set_new_pose_;
  bool isAngleConsidirable = angle_in_radian > angle_diff_to_set_new_pose_;

  return (isDistConsidirable || isAngleConsidirable) ? true : false;
}
