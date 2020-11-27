#include "move_base_msgs/MoveBaseGoal.h"
#include "ros/time.h"
#include "tf2_ros/transform_listener.h"
#include <actionlib/client/simple_action_client.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Quaternion.h>
#include <geometry_msgs/TransformStamped.h>
#include <geometry_msgs/Vector3.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <ros/ros.h>

namespace Follower {

struct RPY {
  double r;
  double p;
  double y;
};

using tfStamped = geometry_msgs::TransformStamped;
using PoseStamped = geometry_msgs::PoseStamped;
using tfListener = tf2_ros::TransformListener;

using QuaternionMsg = geometry_msgs::Quaternion;
using QuaternionTf = tf2::Quaternion;

using Vector3 = geometry_msgs::Vector3;
using MoveBaseGoal = move_base_msgs::MoveBaseGoal;
using MoveBaseClient = actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction>;

class ObjectFollower {
public:
  ObjectFollower();
  auto follow() -> void;

private:
  auto sendGoal(const MoveBaseGoal &goal) noexcept -> void;
  auto checkTf() const -> void;
  auto showGoalState() const noexcept -> void;
  auto evalTranslationalGoal(Vector3 &point, const double yaw) -> void;

  auto getObjTf() const -> tfStamped;
  auto evalTfGoal(tfStamped &pose) -> void;

  static auto evalRotationalGoal(tfStamped &pose, const double yaw) -> void;
  static auto getYawFromQuaternion(const QuaternionMsg &q) noexcept -> double;
  static auto tfToGoal(const tfStamped &pose) -> MoveBaseGoal;

private:
  static const inline ros::Time tf_oldness_ = ros::Time(0);
  static const inline ros::Duration tf_wait_ = ros::Duration(5.0);

  std::unique_ptr<tfListener> tf_listener_;
  MoveBaseClient move_base_client_ = MoveBaseClient("move_base", true);

  tf2_ros::Buffer tf_buffer_;
  tfStamped current_position_;

protected:
  std::string base_frame_;
  std::string object_frame_;

  double range_diff_to_set_new_pose_ = 1.0;
  double yaw_diff_to_set_new_pose_ = 20.0;
  double max_dist_to_obj_ = 5.0;
  double goal_dist_from_obj_ = 1.0;
};

}; // namespace Follower
