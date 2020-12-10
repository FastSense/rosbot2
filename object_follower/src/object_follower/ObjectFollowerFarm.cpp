#include "ObjectFollowerFarm.hpp"

ObjectFollowerFarm::ObjectFollowerFarm() : pnh_("~") {}

std::unique_ptr<GoalGenerator> ObjectFollowerFarm::makeGenerator(std::string_view type) {
  std::unique_ptr<GoalGenerator> generator;
  if (type == generator_types[0])
    generator = std::make_unique<GoalGeneratorNearest2D>();
  else if (type == generator_types[1])
    generator = std::make_unique<GoalGeneratorBaselink2D>();

  return generator;
}

std::unique_ptr<GoalPublisher> ObjectFollowerFarm::makePublisher(std::string_view type) {
  std::unique_ptr<GoalPublisher> publisher;
  if (type == publisher_types[0])
    publisher = std::make_unique<GoalPublisherTf>();
  else if (type == publisher_types[1])
    publisher = std::make_unique<GoalPublisherMoveBase>();

  return publisher;
}

std::unique_ptr<GoalChecker> ObjectFollowerFarm::makeChecker(std::string_view type) {
  std::unique_ptr<GoalChecker> checker;
  if (type == checker_types[0])
    checker = std::make_unique<GoalChecker>();

  return checker;
}

ObjectFollowerCore ObjectFollowerFarm::makeFollower(std::string_view checker_type,
                                                    std::string_view publisher_type,
                                                    std::string_view generator_type) {
  auto generator = makeGenerator(generator_type);
  auto publisher = makePublisher(publisher_type);
  auto checker = makeChecker(checker_type);

  return ObjectFollowerCore(std::move(checker), std::move(generator), std::move(publisher));
}

ObjectFollowerCore ObjectFollowerFarm::makeFollower() {
  return makeFollower(current_checker_, curren_publisher_, current_generator_);
}

void ObjectFollowerFarm::setParams() {
  pnh_.param<std::string>("generator_type", current_generator_, "nearest_2d");
  pnh_.param<std::string>("publisher_type", curren_publisher_, "tf");
  pnh_.param<std::string>("cheker_type", current_checker_, "default");
}
