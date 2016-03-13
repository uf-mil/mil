#include <sub8_pcl/cv_tools.hpp>

namespace sub {
cv::Point contour_centroid(Contour& contour) {
  cv::Moments m = cv::moments(contour, true);
  cv::Point center(m.m10 / m.m00, m.m01 / m.m00);
  return center;
}

ImageWithCameraInfo::ImageWithCameraInfo(sensor_msgs::ImageConstPtr _image_msg,
										 sensor_msgs::CameraInfoConstPtr _info_msg)
	: image_msg(_image_msg), info_msg(_info_msg), image_time(_image_msg->header.stamp) {}


FrameHistory::FrameHistory(std::string img_topic, unsigned int hist_size)
	: topic_name(img_topic), history_size(hist_size), _image_transport(nh), frame_count(0) 
{
	std::stringstream console_msg;
	console_msg << "[FrameHistory] size set to " << history_size << std::endl
				<< "\tSubscribing to image topic: " << topic_name << std::endl;
	ROS_INFO(console_msg.str().c_str());
	_image_sub = _image_transport.subscribeCamera(img_topic, 1, &FrameHistory::image_callback, this);
	if(_image_sub.getNumPublishers() == 0){
		std::stringstream error_msg;
		error_msg << "[FrameHistory] no publishers currently publishing to " << topic_name << std::endl;
		ROS_WARN(error_msg.str().c_str());
	}
}


FrameHistory::~FrameHistory(){
	std::stringstream console_msg;
	console_msg << "[FrameHistory] Unsubscribed from image topic: " << topic_name << std::endl
			    << "[FrameHistory] Deleting FrameHistory object" << std::endl;
	ROS_INFO(console_msg.str().c_str());
}


void FrameHistory::image_callback(const sensor_msgs::ImageConstPtr &image_msg, const sensor_msgs::CameraInfoConstPtr &info_msg){
	/**
		Adds an  ImageWithCameraInfo object to the frame history ring buffer
	*/
	ImageWithCameraInfo current_frame(image_msg, info_msg);
	bool full = _frame_history_ring_buffer.size() >= history_size;
	std::stringstream debug_msg;
	debug_msg << "Adding frame to ring buffer "
			  << "[frame=" << frame_count << "," << "full=" << (full? "true" : "false")
			  << ",frames_available=" << _frame_history_ring_buffer.size() << "]" << std::endl;
	ROS_DEBUG(debug_msg.str().c_str());
	if(!full){
		_frame_history_ring_buffer.push_back(current_frame);
	}
	else {
		_frame_history_ring_buffer[frame_count % history_size] = current_frame;
	}
	frame_count++;
}


std::vector<ImageWithCameraInfo> FrameHistory::get_frame_history(unsigned int frames_requested){
	/**
		Returns a vector with the last <num_frames> ImageWithCameraInfo objects
	*/
	std::vector<ImageWithCameraInfo> frame_history;
	std::vector<ImageWithCameraInfo> sorted_frame_history = _frame_history_ring_buffer;
	if(_frame_history_ring_buffer.size() < frames_requested){
		ROS_WARN("get_frame_history(%d): %d frames were requested, but there are %d frames available",
				 frames_requested, frames_requested, _frame_history_ring_buffer.size());
	}
	else{
		std::sort(sorted_frame_history.begin(), sorted_frame_history.end());
		for(size_t i = 0; i < frames_requested; i++){
			frame_history.push_back(sorted_frame_history[i]);
			if(i == frames_requested - 1) break;
		}
	}
	return frame_history;
}

} // namespace sub