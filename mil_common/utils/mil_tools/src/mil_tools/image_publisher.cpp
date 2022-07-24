#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <opencv2/highgui/highgui.hpp>
#include <cv_bridge/cv_bridge.h>

int main(int argc, char** argv)
{
	ros::init(argc, argv, "init_publisher");
	ros::NodeHandle nh;
	image_transport:: ImageTransport it(nh);
	image_transport::Publisher pub = it.advertise("camera/image", 1);
	cv::Mat image = cv::imread(argv[1], CV_LOAD_IMAGE_COLOR);
	cv::waitKey(30);
	sensor_msgs::ImagePtr msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", image).toImageMsg();

	ros::Rate loop_rate(5);

	while (nh.ok()){

		pub.publish(msg);
		ros::spinOnce();
		loop_rate.sleep();
	}




}

class Image_Publisher
{


  public:
	Image_Publisher(const std::string & topic) : it(nh)
	{
		ros::init(argc, argv, "init_publisher");
        	ros::NodeHandle nh;
        	image_transport:: ImageTransport it(nh);
        	image_transport::Publisher pub = it.advertise(topic, 1);
	}
	void publish(cv_bridge::CvImage imagearg)
	{
		cv::Mat image = cv::imread(imagearg, CV_LOAD_IMAGE_COLOR);
        	cv::waitKey(30);
       		sensor_msgs::ImagePtr msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", image).toImageMsg();

        	ros::Rate loop_rate(5);

        	while (nh.ok()){

                	pub.publish(msg);
                	ros::spinOnce();
                	loop_rate.sleep();
        	}

	}
}
