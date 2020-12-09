#pragma once

#include "geometry_msgs/TransformStamped.h"
#include <optional>

class GoalChecker {
public:
  GoalChecker() = default;
  virtual ~GoalChecker() = default;
  bool updatePoseIfConsiderable(const geometry_msgs::TransformStamped &pose);

  double angle_diff_to_set_new_pose_;
  double range_diff_to_set_new_pose_;

protected:
  bool isGoalConsiderable(const geometry_msgs::TransformStamped &pose) const;

  inline void setCurrentPosition(const geometry_msgs::TransformStamped &pose) {
    current_position_ = pose;
  };

protected:
  std::optional<geometry_msgs::TransformStamped> current_position_;
};
