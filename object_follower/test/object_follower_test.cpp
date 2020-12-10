#include <gtest/gtest.h>
#include <ros/ros.h>


/// @brief Test Fixture
class ObjectFollowerTest : public ::testing::Test {
protected:
  void SetUp() override {}
  void TearDown() override {}
};

TEST_F(ObjectFollowerTest, Templatetest) {
}


int main(int argc, char **argv) {
  testing::InitGoogleTest(&argc, argv);
  ros::init(argc, argv, "ObjectFollowerTest");

  return RUN_ALL_TESTS();
}
