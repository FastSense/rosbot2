#pragma once

#include "GoalGenerator.hpp"

class GoalGeneratorNearest : public GoalGenerator {
public:
  GoalGeneratorNearest() = default;
  virtual ~GoalGeneratorNearest() = default;
  void evalGoal(geometry_msgs::TransformStamped &pose) final;
};
