
// gtest
#include <gtest/gtest.h>

// STD
#include <vector>

#include <iostream>
// #include <ros/ros.h>

// Declare another test
TEST(TestSuite, testCase2)
{
    std::cout << "Trackkk " << std::endl;
    EXPECT_EQ(true, true);
}

// Run all the tests that were declared with TEST()
int main(int argc, char **argv){
  testing::InitGoogleTest(&argc, argv);
//   ros::init(argc, argv, "tester");
//   ros::NodeHandle nh;
  return RUN_ALL_TESTS();
}