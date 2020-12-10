#pragma once

#include "GoalGenerator.hpp"

class GoalGeneratorNearest2D : public GoalGenerator {
public:
  GoalGeneratorNearest2D() = default;
  virtual ~GoalGeneratorNearest2D() = default;
  void evalGoal(geometry_msgs::TransformStamped &pose) final;
};
