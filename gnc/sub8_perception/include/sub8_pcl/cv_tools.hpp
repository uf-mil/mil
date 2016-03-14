#pragma once

#include <iostream>
#include <string>
#include <sstream>
#include <vector>
#include <algorithm>

#include <opencv2/opencv.hpp>

#include <image_geometry/pinhole_camera_model.h>
#include <image_transport/image_transport.h>
#include <sensor_msgs/image_encodings.h>
#include "ros/ros.h"

namespace sub {

typedef std::vector<cv::Point> Contour;

// Compute the centroid of an OpenCV contour (Not templated)
cv::Point contour_centroid(Contour& contour);

struct ImageWithCameraInfo
{	
	/**
		Packages corresponding  sensor_msgs::ImageConstPtr and sensor_msgs::CameraInfoConstPtr info_msg
		into one object. Containers of these objects can be sorted by their image_time attribute
	*/
	public:
		ImageWithCameraInfo(sensor_msgs::ImageConstPtr image_msg, sensor_msgs::CameraInfoConstPtr info_msg);
		sensor_msgs::ImageConstPtr image_msg;
		sensor_msgs::CameraInfoConstPtr info_msg;
		ros::Time image_time;
		bool operator <(const ImageWithCameraInfo& right) const {
			return this->image_time < right.image_time;
		}
};


class FrameHistory
{
	/**
		Object that subscribes itself to an image topic and stores up to a user defined
		number of ImageWithCameraInfo objects. The frame history can then be retrieved 
		in whole or just a portion.
	*/
public:

	FrameHistory(std::string img_topic, unsigned int hist_size);
	~FrameHistory();
	void image_callback(const sensor_msgs::ImageConstPtr &image_msg, const sensor_msgs::CameraInfoConstPtr &info_msg);
	std::vector<ImageWithCameraInfo> get_frame_history(unsigned int frames_requested);
	int frames_available();

	const std::string topic_name;
	const size_t history_size;

private:
	ros::NodeHandle nh;
	image_transport::CameraSubscriber _image_sub;
	image_transport::ImageTransport _image_transport;
	std::vector<ImageWithCameraInfo> _frame_history_ring_buffer;
	size_t frame_count;

};

} // namespace sub