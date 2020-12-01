#include "object_follower_tf_broadcaster.hpp"
#include "ros/init.h"

namespace Follower {

class TfBroadcasterNode : public TfBroadcaster {
public:
  TfBroadcasterNode();
  virtual auto start() -> void override;
  virtual auto sleep() -> void override;

  virtual ~TfBroadcasterNode() = default;

private:
  ros::Rate node_rate_;
};

} // namespace Follower
