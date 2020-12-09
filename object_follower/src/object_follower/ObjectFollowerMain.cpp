#include "GoalChecker.hpp"

#include "GoalGeneratorBaselink2D.hpp"
#include "GoalPublisherTf.hpp"

#include "ObjectFollowerCore.hpp"
#include "ros/init.h"
#include <memory>

int main(int argc, char **argv) {
  ros::init(argc, argv, "object_follower");

  auto checker = std::make_unique<GoalChecker>();
  auto generator = std::make_unique<GoalGeneratorBaselink2D>();
  auto publisher = std::make_unique<GoalPublisherTf>();

  ObjectFollowerCore node(std::move(checker), std::move(generator),
                                          std::move(publisher));

  ROS_INFO("Object Follower Start Working");
  node.start();
}
