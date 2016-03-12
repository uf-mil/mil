#include <string>
#include <vector>
#include <iostream>

#include <algorithm>
#include <opencv2/opencv.hpp>
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
#include <image_geometry/pinhole_camera_model.h>

#include <tf/transform_listener.h>
#include <sensor_msgs/image_encodings.h>

#include <eigen_conversions/eigen_msg.h>
#include <Eigen/StdVector>
#include <sub8_pcl/pcl_tools.hpp>
#include <sub8_pcl/cv_tools.hpp>

#include "ros/ros.h"

#include "sub8_msgs/VisionRequest.h"

// class FrameHistory
// {
// 	FrameHistory(std::string img_topic, int hist_size);
// 	~FrameHistory();
// 	std::vector<cv::Mat> color;
// 	std::vector<cv::Mat> r;
// 	std::vector<cv::Mat> g;
// 	std::vector<cv::Mat> b;
// 	std::vector<cv::Mat> hsv;
// 	const std::string topic_name;
// 	const int history_size;

// };

// FrameHistory::FrameHistory(std::string img_topic, int hist_size) : topic_name(img_topic), history_size(hist_size){
// 	std::cout << "Initalized: " << this.topic_name << ' ' << this.hist_size << std::endl;
// }

class Sub8TorpedoBoardDetector {
public:
	Sub8TorpedoBoardDetector();
	~Sub8TorpedoBoardDetector();
	void image_callback(const sensor_msgs::ImageConstPtr &msg,
                      const sensor_msgs::CameraInfoConstPtr &info_msg);
	void determine_torpedo_board_position(const image_geometry::PinholeCameraModel &cam_model,
                               const cv::Mat &image_raw);
	bool request_torpedo_board_position(sub8_msgs::VisionRequest::Request &req,
                             sub8_msgs::VisionRequest::Response &resp);
	

	ros::NodeHandle nh;

	image_transport::CameraSubscriber image_sub();	// ?? gcc complains if I remove parentheses
	image_transport::ImageTransport image_transport();
	image_geometry::PinholeCameraModel cam_model();

};

Sub8TorpedoBoardDetector::Sub8TorpedoBoardDetector(){
}

Sub8TorpedoBoardDetector::~Sub8TorpedoBoardDetector(){

}

void Sub8TorpedoBoardDetector::image_callback(const sensor_msgs::ImageConstPtr &msg,
                    					 const sensor_msgs::CameraInfoConstPtr &info_msg){

}

void Sub8TorpedoBoardDetector::determine_torpedo_board_position(const image_geometry::PinholeCameraModel &cam_model,
                            							   const cv::Mat &image_raw){

}

bool Sub8TorpedoBoardDetector::request_torpedo_board_position(sub8_msgs::VisionRequest::Request &req,
                            							 sub8_msgs::VisionRequest::Response &resp){
	return true;
}

int main(int argc, char **argv) {
	ros::init(argc, argv, "torpedo_board_detector");
	boost::shared_ptr<Sub8TorpedoBoardDetector> sub8_torpedo_board(new Sub8TorpedoBoardDetector());
	ros::spin();
}