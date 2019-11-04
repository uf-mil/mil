#include <gtest/gtest.h>
#include <mil_blueview_driver/BlueViewPing.h>
#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <memory>
#include <string>

/* Disabled because test data does not support range profiling
TEST(blueview_driver, GetRawMsg) {
    ros::NodeHandle nh;
    auto msg = ros::topic::waitForMessage<blueview_driver::BlueViewPing>("/test_blue_view/ranges", nh,
ros::Duration(5));
    EXPECT_FALSE(msg == nullptr) << "message not received after 5 seconds";
}
*/
TEST(blueview_driver, GetGrayscaleImageMsg)
{
  ros::NodeHandle nh;
  auto msg = ros::topic::waitForMessage<sensor_msgs::Image>("/test_blue_view/image_mono", nh, ros::Duration(5));
  EXPECT_FALSE(msg == nullptr) << "message not received after 5 seconds";
}
TEST(blueview_driver, GetColorImageMsg)
{
  ros::NodeHandle nh;
  auto msg = ros::topic::waitForMessage<sensor_msgs::Image>("/test_blue_view/image_color", nh, ros::Duration(5));
  EXPECT_FALSE(msg == nullptr) << "message not received after 5 seconds";
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "blueview_driver_test");
  //~ test_class.reset(new BlueViewDriverTest());
  std::srand(unsigned(std::time(0)));
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
