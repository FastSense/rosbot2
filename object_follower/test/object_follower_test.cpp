#include "object_follower_tf_broadcaster_node.hpp"
#include <gtest/gtest.h>
#include <ros/ros.h>

constexpr size_t MAX_ITER_NUM = 10;

class ObjectFollowerTest : public ::testing::Test, public Follower::TfBroadcasterNode {
protected:
  void SetUp() override {}
  void TearDown() override {}
};

TEST_F(ObjectFollowerTest, testLookupTf) {
  bool state = false;

  int i = 0;
  while (i < MAX_ITER_NUM) {
    try {
      getTf();
      state = true;
      break;
    } catch (...) {
      ROS_INFO("Cant get tf: Iteration %d", i);
    }
  }
  EXPECT_TRUE(state);
}

TEST_F(ObjectFollowerTest, testGoalIsGood) {
  Follower::tfStamped position;
  position.transform.translation.x = 0;
  position.transform.translation.y = 0;
  position.transform.translation.z = 0;
  /* current_position_ = */
  /* EXPECT_TRUE(); */
}

int main(int argc, char **argv) {
  testing::InitGoogleTest(&argc, argv);
  ros::init(argc, argv, "ObjectFollowerTest");

  return RUN_ALL_TESTS();
}
