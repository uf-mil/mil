
#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <opencv2/highgui/highgui.hpp>
#include <cv_bridge/cv_bridge.h>


class Image_Publisher
{

  private:
	std::string encoding;
	image_transport::Publisher pub;
  public:
	Image_Publisher(const std::string & topic, const std::string &encoding = "bgr8",int queue_size = 1);
	void publish(cv::Mat imagearg);
};

