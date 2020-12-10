#pragma once

#include "Converter.hpp"
#include "GoalGenerator.hpp"

class GoalGeneratorBaselink2D : public GoalGenerator {
public:
  GoalGeneratorBaselink2D() = default;
  virtual ~GoalGeneratorBaselink2D() = default;

  void evalGoal(geometry_msgs::TransformStamped &pose) final;

private:
  void setGoalRotation(geometry_msgs::Quaternion &rotation, RPY rpy) const;
  void setGoalTranslation(geometry_msgs::Vector3 &pt, RPY rpy) const;
};
