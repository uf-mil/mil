#include <mil_blueview_driver/BlueViewPing.h>
#include <bvt_sdk.h>
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
#include <ros/console.h>
#include <ros/ros.h>
#include <ctime>
#include <functional>
#include <opencv2/core/core.hpp>
#include <stdexcept>
#include <blueview_wrapper.hpp>

class BlueViewRosDriver
{
public:
  BlueViewRosDriver();
  void initParams();
  double period_seconds;
  void run();
private:
  ros::NodeHandle nh;
  BlueViewSonar sonar;
  void loop(const ros::TimerEvent&);
  void get_ping();

  image_transport::ImageTransport image_transport;
  image_transport::Publisher grayscale_pub, color_pub;
  ros::Publisher raw_pub;
  cv_bridge::CvImagePtr grayscale_img, color_img;
  mil_blueview_driver::BlueViewPingPtr ping_msg;

  ros::Timer timer;
  bool do_grayscale, do_color, do_raw;
  std::string frame_id;
};
