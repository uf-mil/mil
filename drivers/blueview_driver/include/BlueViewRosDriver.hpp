#include <bvt_sdk.h>
#include <ros/ros.h>
#include <ros/console.h>
#include <opencv2/core/core.hpp> 
#include <cv_bridge/cv_bridge.h>
#include <functional>
#include <image_transport/image_transport.h>
#include <sonar/BlueViewPing.h>
#include <ctime>
#include <stdexcept>
#include "BVWrapper.hpp"

class BlueViewRosDriver
{
private:
	ros::NodeHandle nh;
	BlueViewSonar sonar;
	void loop(const ros::TimerEvent&);

	image_transport::ImageTransport image_transport;
	image_transport::Publisher grayscale_pub, color_pub;
	ros::Publisher raw_pub;
	cv_bridge::CvImagePtr grayscale_img, color_img;
	sonar::BlueViewPingPtr ping_msg;
	size_t i = 0;
	
	ros::Timer timer;
	bool do_grayscale, do_color, do_raw;
	std::string frame_id;
public:
	BlueViewRosDriver();
	void initParams();
};
