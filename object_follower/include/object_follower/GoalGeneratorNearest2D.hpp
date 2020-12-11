#pragma once

#include "GoalGenerator.hpp"

/**
 * @brief  Implementation for 2D case, in which goal is selected as nearest point between base and object.
 */
class GoalGeneratorNearest2D : public GoalGenerator {
public:
  GoalGeneratorNearest2D() = default;
  virtual ~GoalGeneratorNearest2D() = default;
  void evalGoal(geometry_msgs::TransformStamped &pose) final;
};
