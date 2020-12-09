#include "object_follower_2d.hpp"
#include "object_follower_tf_broadcaster_node.hpp"
#include "tf2/convert.h"
#include <gtest/gtest.h>

#include <ros/ros.h>
#include <vector>

constexpr size_t MAX_LOOKUP_ITER_NUM = 2;
constexpr size_t MAX_FOLLOW_ITER_NUM = 2;

std::vector<double> getTranslationRange(double start, double end, double step) {
  std::vector<double> translations;

  for (double pose_value = start; pose_value < end; pose_value += step) {
    translations.push_back(pose_value);
  }

  return translations;
}

std::vector<double> getRotationRange(double start, double end, double step) {
  std::vector<double> angles;

  for (double angle_value = start; angle_value < end; angle_value += step) {
    double angle_val_in_radian = angle_value * M_PI / 180;
    angles.push_back(angle_val_in_radian);
  }
  return angles;
}

void setYawAngle(Follower::Quaternion &rotation, double yaw) {
  Follower::QuaternionTf q_tf;
  q_tf.setRPY(0, 0, yaw);
  tf2::convert(q_tf, rotation);
}

Follower::Vector3 getPosition(double x, double y, double z) {
  Follower::Vector3 poseVec;
  poseVec.x = x;
  poseVec.y = y;
  poseVec.z = z;

  return poseVec;
}

/// @brief Test Fixture
class ObjectFollowerTest : public ::testing::Test, public Follower::TfBroadcasterFollowerNode {
protected:
  void SetUp() override {}
  void TearDown() override {}
};

TEST_F(ObjectFollowerTest, testLookupTf) {
  bool state = false;

  int i = 0;
  while (i < MAX_LOOKUP_ITER_NUM) {
    try {
      getTf();
      state = true;
      break;
    } catch (...) {
      i++;
    }
  }
  EXPECT_TRUE(state);
}

TEST_F(ObjectFollowerTest, testGoalIsBad) {
  Follower::tfStamped position;

  range_diff_to_set_new_pose_ = 1;
  angle_diff_to_set_new_pose_ = 10;

  position.transform.translation = getPosition(0, 0, 0);
  setYawAngle(position.transform.rotation, 0);
  setCurrentPosition(position);

  auto translations = getTranslationRange(0, 0.95, 0.2);
  auto angles = getRotationRange(0, 9.95, 0.2);

  for (double translation : translations) {
    for (double angle : angles) {
      position.transform.translation.x = translation;
      setYawAngle(position.transform.rotation, angle);
      if (isGoalConsiderable(position))
        FAIL() << "Goal must be BAD \n translation sent: " << translation
               << "\n Rotation sent: " << angle;
    }
  }
}

TEST_F(ObjectFollowerTest, testGoalIsGood) {
  Follower::tfStamped position;

  range_diff_to_set_new_pose_ = 1;
  angle_diff_to_set_new_pose_ = 10;

  position.transform.translation = getPosition(0, 0, 0);
  setYawAngle(position.transform.rotation, 0);
  setCurrentPosition(position);

  auto translations = getTranslationRange(1.05, 3.05, 0.1);
  auto angles = getRotationRange(10.5, 20.5, 0.1);

  for (double translation : translations) {
    for (double angle : angles) {
      position.transform.translation.x = translation;
      setYawAngle(position.transform.rotation, angle);

      if (!isGoalConsiderable(position))
        FAIL() << "Goal must be GOOD \n translation sent: " << translation
               << "\n Rotation sent: " << angle;
    }
  }
}

TEST_F(ObjectFollowerTest, testFollow) {
  int i = 0;
  bool state = false;

  while (i < MAX_FOLLOW_ITER_NUM) {
    try {
      follow();
      state = true;
      return;
    } catch (...) {
      i++;
      exceptionFilter();
    }
  }

  EXPECT_TRUE(state);
}

int main(int argc, char **argv) {
  testing::InitGoogleTest(&argc, argv);
  ros::init(argc, argv, "ObjectFollowerTest");

  return RUN_ALL_TESTS();
}
