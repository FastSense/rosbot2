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

TEST_F(ObjectFollowerTest, testTfGoals) {
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

int main(int argc, char **argv) {
  testing::InitGoogleTest(&argc, argv);
  ros::init(argc, argv, "tester");
  ros::NodeHandle nh;
  return RUN_ALL_TESTS();
}
