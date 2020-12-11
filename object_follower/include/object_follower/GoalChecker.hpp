#pragma once

#include "geometry_msgs/TransformStamped.h"
#include <optional>

/**
 * @brief Class that keeps current position state and checks if goal is considirable.
 */
class GoalChecker {
public:
  GoalChecker() = default;
  virtual ~GoalChecker() = default;

  virtual bool isGoalConsiderable(const geometry_msgs::TransformStamped &pose) const;

  double angle_diff_to_set_new_pose_;
  double range_diff_to_set_new_pose_;

  void setCurrentPosition(const geometry_msgs::TransformStamped &pose) {
    current_position_ = pose;
  };

private:
  std::optional<geometry_msgs::TransformStamped> current_position_;
};
