#include "ObjectFollowerFactory.hpp"

ObjectFollowerFactory::ObjectFollowerFactory() : pnh_("~") {}

std::unique_ptr<GoalGenerator> ObjectFollowerFactory::makeGenerator(std::string_view type) {
  std::unique_ptr<GoalGenerator> generator;
  if (type == generator_types[0])
    generator = std::make_unique<GoalGeneratorNearest2D>();
  else if (type == generator_types[1])
    generator = std::make_unique<GoalGeneratorBaselink2D>();
  else
    printGeneratorMissingMessage(type);

  return generator;
}

std::unique_ptr<GoalPublisher> ObjectFollowerFactory::makePublisher(std::string_view type) {
  std::unique_ptr<GoalPublisher> publisher;
  if (type == publisher_types[0])
    publisher = std::make_unique<GoalPublisherTf>();
  else if (type == publisher_types[1])
    publisher = std::make_unique<GoalPublisherMoveBase>();
  else
    printPublisherMissingMessage(type);

  return publisher;
}

std::unique_ptr<GoalChecker> ObjectFollowerFactory::makeChecker(std::string_view type) {
  std::unique_ptr<GoalChecker> checker;
  if (type == checker_types[0])
    checker = std::make_unique<GoalChecker>();
  else
    printChekerMissingMessage(type);

  return checker;
}

std::optional<ObjectFollowerCore>
ObjectFollowerFactory::makeFollower(std::string_view checker_type, std::string_view publisher_type,
                                    std::string_view generator_type) {
  std::optional<ObjectFollowerCore> follower;

  auto generator = makeGenerator(generator_type);
  auto publisher = makePublisher(publisher_type);
  auto checker = makeChecker(checker_type);

  if (generator && publisher && checker)
    follower.emplace(
        ObjectFollowerCore(std::move(checker), std::move(generator), std::move(publisher)));

  return follower;
}

std::optional<ObjectFollowerCore> ObjectFollowerFactory::makeFollower() {
  return makeFollower(current_checker_, curren_publisher_, current_generator_);
}

void ObjectFollowerFactory::printGeneratorMissingMessage(std::string_view generator) {
  std::string generators_string;
  for (const auto &gen_type : generator_types) {
    generators_string += gen_type;
    generators_string += " ";
  }
  ROS_ERROR("Wrong generator type: %s. You should choose one of the following : %s",
            generator.data(), generators_string.c_str());
}

void ObjectFollowerFactory::printPublisherMissingMessage(std::string_view publisher) {
  std::string publishers_string;
  for (const auto &pub_type : publisher_types) {
    publishers_string += pub_type;
    publishers_string += " ";
  }
  ROS_ERROR("Wrong publisher type: %s. You should choose one of the following : %s",
            publisher.data(), publishers_string.c_str());
}

void ObjectFollowerFactory::printChekerMissingMessage(std::string_view checker) {
  std::string checkers_string;
  for (const auto &chk_type : checker_types) {
    checkers_string += chk_type;
    checkers_string += " ";
  }
  ROS_ERROR("Wrong cheker type: %s. You should choose one of the following : %s", checker.data(),
            checkers_string.data());
}

void ObjectFollowerFactory::setParams() {
  pnh_.param<std::string>("generator_type", current_generator_, "nearest_2d");
  pnh_.param<std::string>("publisher_type", curren_publisher_, "tf");
  pnh_.param<std::string>("cheker_type", current_checker_, "default");
}
