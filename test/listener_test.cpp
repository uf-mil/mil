#include <ros_alarms/listener.hpp>
#include <iostream>
#include <gtest/gtest.h>

using namespace std;

class ListenerTest : public ::testing::Test
{
public:
  ListenerTest()
  : al(nh, "kill")
  {
    std::cerr << "ros_alarms integration test" << std::endl;
    int i = 3;
    EXPECT_EQ(i, 3);
    EXPECT_EQ(i, 4);
  }
private:
  ros::NodeHandle nh;
  ros_alarms::AlarmListener<> al;
};

TEST_F(ListenerTest, testStuff)
{
  cerr << "Hey!" << endl;
  return;
}
