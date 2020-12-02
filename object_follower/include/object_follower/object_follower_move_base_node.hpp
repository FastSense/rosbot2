#include "object_follower_move_base.hpp"
#include "ros/init.h"

namespace Follower {

class MoveBaseFollowerNode : public MoveBaseFollower {
public:
  MoveBaseFollowerNode();
  auto start() -> void final;
  auto sleep() -> void final;

  ~MoveBaseFollowerNode() = default;

private:
  ros::Rate node_rate_;
};

} // namespace Follower
