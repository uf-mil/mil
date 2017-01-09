#include <bvt_sdk.h>
#include <ros/ros.h>
#include <ros/console.h>
#include <opencv2/core/core.hpp> 
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>

#include <ctime>

#include "BVWrapper.h"


int main(int argc, char** argv){
	try{
		ros::init(argc, argv, "sonar_image_publisher");
		ros::NodeHandle nh;
		image_transport::ImageTransport it(nh);
		image_transport::Publisher pub_bw = it.advertise("sonar/mono8", 1);
		image_transport::Publisher pub_bw16 = it.advertise("sonar/mono16", 1);
		image_transport::Publisher pub_color = it.advertise("sonar/color", 1);

		const char host[] = "192.168.37.53";

		bool sonar_initialized = false;
		std::unique_ptr<bv::BVWrapper> sonar;


		ros::Rate loop_rate_init(1);
		while(!sonar_initialized){
			try{
				sonar = std::unique_ptr<bv::BVWrapper>(new bv::BVWrapper(bv::DEVICE, host));
				sonar_initialized = true;
			}catch(char const* msg){
				std::cout << "ERROR: " << msg << std::endl;
				std::cout << "Failed to initialize head, check tether connection"  << std::endl;
			}
			loop_rate_init.sleep();
		}

		cv::Mat imagebw8;
		cv::Mat imagebw16;
		cv::Mat imagecolor;
		sensor_msgs::ImagePtr msgColor;
		sensor_msgs::ImagePtr msgBW;
		sensor_msgs::ImagePtr msgBW16;

		int slider = 40;
		cv::namedWindow("Sonar Config", 1);
     	cv::createTrackbar("Max Distance (m)", "Sonar Config", &slider, 100);

     	bool pings_not_received = false;
     	int old_slider = slider;
     	int count = 0;

		while (nh.ok()) {
			std::clock_t begin = std::clock();
			if(pings_not_received){
				try{
					sonar->reinitializeSonar();
				}catch(char const* msg){
					std::cout << "ERROR: " << msg << std::endl;
					std::cout << "Failed to initialize head, check tether connection"  << std::endl;
					continue;
				}
			}

			if(!sonar->getNextPing(imagebw8, imagebw16, imagecolor)){
				std::cout << "Ping Not Received" << std::endl;
				ROS_DEBUG_NAMED("WARNING", "The ping was not received");
				pings_not_received = true;
				continue;
			}else{
				pings_not_received = false;
			}

			


			if(old_slider != slider)
				sonar->setStopRange((float)slider);

			old_slider = slider;
			cv::imshow("Sonar Config", imagecolor);
			if(cv::waitKey(30) >= 0) break;
			msgColor = cv_bridge::CvImage(std_msgs::Header(), "bgra8", imagecolor).toImageMsg();
			msgBW = cv_bridge::CvImage(std_msgs::Header(), "mono8", imagebw8).toImageMsg();
			msgBW16 = cv_bridge::CvImage(std_msgs::Header(), "mono16", imagebw16).toImageMsg();
			pub_bw.publish(msgBW);
			pub_bw16.publish(msgBW16);
			pub_color.publish(msgColor);
			ros::spinOnce();
			std::clock_t end = std::clock();
  			double elapsed_secs = double(end - begin) / CLOCKS_PER_SEC;
  			// std::cout << count++ << " ELAPSED: " << elapsed_secs << std::endl;
			// if(count == 10){
			// 	break;
			// }
		}		
	}catch(char const* msg){
		std::cout << "ERROR: " << msg << std::endl << std::endl;
	}

	return 0;
}