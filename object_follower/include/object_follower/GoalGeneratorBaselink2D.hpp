#pragma once

#include "ConversionFunctions.hpp"
#include "GoalGenerator.hpp"

class GoalGeneratorBaselink2D : public GoalGenerator {
public:
  GoalGeneratorBaselink2D() = default;
  virtual ~GoalGeneratorBaselink2D() = default;

  virtual void evalGoalTf(geometry_msgs::TransformStamped &pose) override;

private:
  void setGoalRotation(geometry_msgs::Quaternion &rotation, RPY rpy) const;
  void setGoalTranslation(geometry_msgs::Vector3 &pt, RPY rpy) const;
};
