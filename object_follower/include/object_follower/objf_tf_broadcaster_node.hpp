#include "objf_tf_broadcaster.hpp"
#include "ros/init.h"

namespace Follower {

/// Implementation of TfBroadcasterFollowerNode abstract class
class TfBroadcasterFollowerNode : public TfBroadcasterFollower {
public:
   /// Initialize TfBroadcasterFollowerNode::node_rate_
  TfBroadcasterFollowerNode();
  void start() final;
  void sleep() final;

  ~TfBroadcasterFollowerNode() = default;

private:
  ros::Rate node_rate_;
};

} // namespace Follower
