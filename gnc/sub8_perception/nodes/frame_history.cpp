#include <string>
#include <sstream>
#include <vector>
#include <iostream>
#include <boost/algorithm/string/replace.hpp>

#include <algorithm>
#include <opencv2/opencv.hpp>
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
#include <image_geometry/pinhole_camera_model.h>

#include <sensor_msgs/image_encodings.h>

#include <eigen_conversions/eigen_msg.h>
#include <Eigen/StdVector>
#include <sub8_pcl/pcl_tools.hpp>
#include <sub8_pcl/cv_tools.hpp>

#include "ros/ros.h"

#include "sub8_msgs/VisionRequest.h"

class FrameHistory
{
public:
	FrameHistory(std::string img_topic, int hist_size);
	~FrameHistory();
	void image_callback(const sensor_msgs::ImageConstPtr &image_msg, const sensor_msgs::CameraInfoConstPtr &info_msg);
	bool add_new_frame(cv::Mat new_frame);
	std::vector<cv::Mat> bgr;
	std::vector<cv::Mat> hsv;
	const std::string topic_name;
	const size_t history_size;

	ros::NodeHandle nh;

	image_transport::CameraSubscriber image_sub;
	image_transport::ImageTransport _image_transport;
	image_geometry::PinholeCameraModel cam_model;
private:
	size_t frame_count = 0;

};

FrameHistory::FrameHistory(std::string img_topic, int hist_size)
	: topic_name(img_topic), history_size(hist_size), _image_transport(nh) 
{
	std::cout << "Subscribed to image topic: " << this->topic_name << '\n'
			  << "Frame history size set to " << this->history_size << std::endl;
	this->image_sub = _image_transport.subscribeCamera(img_topic, 1, &FrameHistory::image_callback, this);
	if(this->image_sub.getNumPublishers() == 0){
		std::stringstream error_msg;
		error_msg << "FrameHistory unable to subscribe to " << this->topic_name << std::endl;
				  // << "Shutting down frame history node" << std::endl;
		ROS_WARN(error_msg.str().c_str());
		// ros::shutdown();
	}
}

FrameHistory::~FrameHistory(){

}

void FrameHistory::image_callback(const sensor_msgs::ImageConstPtr &image_msg, const sensor_msgs::CameraInfoConstPtr &info_msg){
	cv::Mat current_image;
	cv_bridge::CvImagePtr input_bridge;
	try {
		input_bridge = cv_bridge::toCvCopy(image_msg, sensor_msgs::image_encodings::BGR8);
		current_image = input_bridge->image;
	} catch (cv_bridge::Exception &ex) {
    	ROS_ERROR("[image_callback] Failed to convert image");
    	return;
	}
	cam_model.fromCameraInfo(info_msg);
	ros::Time image_time = image_msg->header.stamp;
	std::cout << image_time << " ";
	this->add_new_frame(current_image);

}

bool FrameHistory::add_new_frame(cv::Mat new_frame){
	cv::Mat hsv_conversion;
	cvtColor(new_frame, hsv_conversion, CV_BGR2HSV);

	// int size = this->history_size;
	bool full = this->bgr.size() >= this->history_size;
	std::cout << "[frame=" << this->frame_count << "," << "full=" << (full? "true" : "false")
			  << ",size=" << this->bgr.size() << "]" << std::endl;
	if(!full){
		this->bgr.push_back(new_frame);
		this->hsv.push_back(hsv_conversion);
	}
	else{
		this->bgr[this->frame_count % this->history_size] = new_frame;
		this->hsv[this->frame_count % this->history_size] = new_frame;
	}
	// std::cout << "loc1" << std::endl;
	// std::cout << "loc2" << std::endl;
	cv::imshow("Added to bgr History", this->bgr[this->frame_count % this->history_size]);
	cv::imshow("Added to hsv History", this->hsv[this->frame_count % this->history_size]);
	cv::waitKey(1);
	this->frame_count++;
	return true;
}


int main(int argc, char** argv){
	std::string usage = "Usage: frame_history <image_topic> <history_size>";
	if(argc != 3){ std::cout << usage << std::endl; return 0;}
	// for(int i = 0; i < argc; i++){
	// 	std::string arg = std::string(argv[i]);
	// 	std::cout << i << ' ' << arg << std::endl;
	// }
	std::string image_topic = std::string(argv[1]);
	int history_size = atoi(argv[2]);
	ros::init(argc, argv, 
			  "frame_history" + boost::replace_all_copy(image_topic, "/", "_") + "_" + argv[2]);
	boost::shared_ptr<FrameHistory> frame_history(new FrameHistory(image_topic, history_size));
	ros::spin();
}