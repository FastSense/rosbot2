#include "GoalGeneratorBaselink2D.hpp"
#include "GoalGeneratorNearest2D.hpp"
#include "GoalPublisherMoveBase.hpp"
#include "GoalPublisherTf.hpp"

#include "GoalChecker.hpp"

#include "ObjectFollowerCore.hpp"

#include "ros/init.h"
#include "ros/node_handle.h"
#include <memory>

#include <array>
#include <string_view>

using namespace std::string_view_literals;

class ObjectFollowerFarm {
public:
  ObjectFollowerFarm();

  static std::unique_ptr<GoalGenerator> makeGenerator(std::string_view type);
  static std::unique_ptr<GoalPublisher> makePublisher(std::string_view type);
  static std::unique_ptr<GoalChecker> makeChecker(std::string_view type);

  ObjectFollowerCore makeFollower(std::string_view generator_type, std::string_view publisher_type,
                                  std::string_view checker_type);

  ObjectFollowerCore makeFollower();

  void setParams();

private:
  std::string current_generator_;
  std::string curren_publisher_;
  std::string current_checker_;

  ros::NodeHandle pnh_;

  constexpr static std::array generator_types = {
      "nearest_2d"sv,
      "base_link_2d"sv,
  };

  constexpr static std::array publisher_types = {
      "tf"sv,
      "move_base"sv,
  };

  constexpr static std::array checker_types = {
      "default"sv,
  };
};
