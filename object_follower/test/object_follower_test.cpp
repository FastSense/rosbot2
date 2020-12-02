#include <gtest/gtest.h>
#include <ros/ros.h>
#include <stdexcept>

TEST(TestSuite, testCase1) { ASSERT_EQ(1, 2); }

TEST(TestSuite, testCase2) { ASSERT_EQ(2, 2); }

TEST(TestSuite, testCase3) {
  try {
    throw std::runtime_error("runtime_error");
  } catch (...) {
    ADD_FAILURE() << "LINE OF CATCH WITH ADD FAILURE";
  }
}

TEST(TestSuite, testCase4) {
  try {
    throw std::runtime_error("runtime_error");
  } catch (...) {
    FAIL() << "LINE OF CATCH WITH FAIL";
  }
}

int main(int argc, char **argv) {
  testing::InitGoogleTest(&argc, argv);
  ros::init(argc, argv, "tester");
  ros::NodeHandle nh;
  return RUN_ALL_TESTS();
}
