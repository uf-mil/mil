#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
#include <ros/ros.h>

#include <opencv2/highgui/highgui.hpp>

class ImagePublisher
{
public:
  ImagePublisher(const std::string& topic, const std::string& encoding = "bgr8", int queue_size = 1);
  void publish(cv::Mat& image_arg);

private:
  std::string encoding_;
  image_transport::Publisher pub_;
  ros::NodeHandle nh_;
  image_transport::ImageTransport it_;
};
