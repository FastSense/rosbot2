#include "GoalChecker.hpp"
#include "GoalGeneratorBaselink2D.hpp"
#include "GoalGeneratorNearest2D.hpp"
#include "GoalPublisherMoveBase.hpp"
#include "GoalPublisherTf.hpp"

#include "ObjectFollowerCore.hpp"

#include "ros/init.h"
#include "ros/node_handle.h"
#include <memory>

#include <array>
#include <string_view>

static const std::array generator_types = {
    std::string_view("nearest_2d"),
    std::string_view("base_link_2d"),
};

static const std::array publisher_types = {
    std::string_view("tf"),
    std::string_view("move_base"),
};

int main(int argc, char **argv) {
  ros::init(argc, argv, "object_follower");

  std::unique_ptr<GoalGenerator> generator;
  std::unique_ptr<GoalPublisher> publisher;
  std::unique_ptr<GoalChecker> checker;

  ros::NodeHandle nh_build("~");

  std::string generator_type;
  std::string publisher_type;

  nh_build.param<std::string>("generator_type", generator_type, "nearest_2d");
  nh_build.param<std::string>("publisher_type", publisher_type, "tf");

  if (generator_type == generator_types[0])
    generator = std::make_unique<GoalGeneratorNearest2D>();
  else if (generator_type == generator_types[1])
    generator = std::make_unique<GoalGeneratorBaselink2D>();

  if (publisher_type == publisher_types[0]) {
    publisher = std::make_unique<GoalPublisherTf>();

  } else if (publisher_type == publisher_types[1])
    publisher = std::make_unique<GoalPublisherMoveBase>();

  checker = std::make_unique<GoalChecker>();

  ROS_INFO("Generator: %s", generator_type.c_str());
  ROS_INFO("Publisher: %s", publisher_type.c_str());
  ROS_INFO("Checker: default");

  ObjectFollowerCore node(std::move(checker), std::move(generator), std::move(publisher));

  ROS_INFO("Object Follower Start Working");
  node.start();
}
