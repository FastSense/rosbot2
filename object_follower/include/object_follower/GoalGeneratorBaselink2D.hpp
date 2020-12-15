#pragma once

#include "Converter.hpp"
#include "GoalGenerator.hpp"

/**
 * @brief  Implementation for 2D case, in which goal is selected opposite the object X coordinate
 * for x - forward, z - top, y - left orientation.
 */
class GoalGeneratorBaselink2D : public GoalGenerator {
public:
  GoalGeneratorBaselink2D() = default;
  virtual ~GoalGeneratorBaselink2D() = default;

  geometry_msgs::TransformStamped evalGoal() final;

private:
  void setGoalRotation(geometry_msgs::Quaternion &rotation, RPY rpy) const;
  void setGoalTranslation(geometry_msgs::Vector3 &pt, RPY rpy) const;
};
