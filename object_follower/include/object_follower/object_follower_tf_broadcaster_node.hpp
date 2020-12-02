#include "object_follower_tf_broadcaster.hpp"
#include "ros/init.h"

namespace Follower {

class TfBroadcasterNode : public TfBroadcaster {
public:
  TfBroadcasterNode();
  void start() final;
  void sleep() final;

  ~TfBroadcasterNode() = default;

private:
  ros::Rate node_rate_;
};

} // namespace Follower
