#include "object_follower_move_base.hpp"
#include "ros/init.h"

namespace Follower {

/**
 * @brief Implementation of MoveBaseFollower abstract class
 */
class MoveBaseFollowerNode : public MoveBaseFollower {
public:
  MoveBaseFollowerNode();
  void start() final;
  void sleep() final;

  ~MoveBaseFollowerNode() = default;

private:
  ros::Rate node_rate_;
};

} // namespace Follower
