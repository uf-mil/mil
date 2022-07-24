#include <mil_tools/image_publisher.h>
/*
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
*/

Image_Publisher::Image_Publisher(const std::string & topic, const std::string &encoding = "bgr8",int queue_size = 1 )
{
	this->encoding = encoding;
	ros::init(argc, argv, "init_publisher");
       	ros::NodeHandle nh;
       	image_transport:: ImageTransport it(nh);
       	pub = it.advertise(topic, queue_size);
}
void Image_Publisher::publish(cv::Mat imagearg)
{
    	sensor_msgs::ImagePtr msg = cv_bridge::CvImage(std_msgs::Header(), encoding, cv::Mat& imagearg).toImageMsg();
        pub.publish(msg);
}

