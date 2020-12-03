#include "object_follower_tf_broadcaster.hpp"
#include "ros/init.h"

namespace Follower {

/// Implementation of TfBroadcaster abstract class
class TfBroadcasterFollowerNode : public TfBroadcasterFollower {
public:
  TfBroadcasterFollowerNode();
  void start() final;
  void sleep() final;

  ~TfBroadcasterFollowerNode() = default;

private:
  ros::Rate node_rate_;
};

} // namespace Follower
