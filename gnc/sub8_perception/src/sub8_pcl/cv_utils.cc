#include <sub8_pcl/cv_tools.hpp>

namespace sub {

cv::Point contour_centroid(Contour& contour) {
  cv::Moments m = cv::moments(contour, true);
  cv::Point center(m.m10 / m.m00, m.m01 / m.m00);
  return center;
}


cv::MatND smooth_histogram(const cv::MatND &histogram, size_t filter_kernel_size, float sigma){
	// Smoothen histogram
	/**
	    Currently only support smoothing using gaussian kernels with sigma = 1.2 of size 3 or 5
	*/

	cv::MatND hist = histogram.clone();
	std::vector<float> gauss_kernel = generate_gaussian_kernel_1D(filter_kernel_size, sigma);
	size_t histSize = hist.total();
	int offset = (filter_kernel_size - 1) / 2;
	for (int i = offset; i < histSize-offset; i++ )   // Convolve histogram values with gaussian kernel
	{
		int sum = 0;
		int kernel_idx = 0;
		for (int j = i - offset; j <= i + offset; j++){
			sum += (hist.at<float>(j) * gauss_kernel[kernel_idx++]);
	    }
	    hist.at<float>(i) = sum;
	}
	for (int i = 0; i < offset; ++i)  // Pad filtered result with zeroes
	{
		hist.at<float>(i) = 0;
		hist.at<float>(histSize - 1 - i) = 0;
	}
	return hist;
}


std::vector<float> generate_gaussian_kernel_1D(size_t kernel_size, float sigma){
	std::vector<float> kernel;
	int middle_index = (kernel_size - 1) / 2;
	int first_discrete_sample_x = -(middle_index);
	for (int i = first_discrete_sample_x; i <= 0; i++)
	{
		float power = -0.5 * (float(i)/sigma) * (float(i)/sigma);
		kernel.push_back( exp(power) );				// From definition of Standard Normal Distribution
	}
	for(int i = 1; i <= middle_index; i++){			// Kernel is symmetric
		kernel.push_back(kernel[middle_index - i]);
	}
	// Normalize kernel (sum of values should equal 1.0)
	float sum = 0;
	for (size_t i = 0; i < kernel_size; i++){ sum += kernel[i];	}
	for (size_t i = 0; i < kernel_size; i++){ kernel[i] /= sum;	}
	return kernel;
}


ImageWithCameraInfo::ImageWithCameraInfo(sensor_msgs::ImageConstPtr _image_msg,
										 sensor_msgs::CameraInfoConstPtr _info_msg)
	: image_msg(_image_msg), info_msg(_info_msg), image_time(_image_msg->header.stamp) {}


FrameHistory::FrameHistory(std::string img_topic, unsigned int hist_size)
	: topic_name(img_topic), history_size(hist_size), _image_transport(nh), frame_count(0) 
{
	std::stringstream console_msg;
	console_msg << "[FrameHistory] size set to " << history_size << std::endl
				<< "\tSubscribing to image topic: " << topic_name;
	ROS_INFO(console_msg.str().c_str());
	_image_sub = _image_transport.subscribeCamera(img_topic, 1, &FrameHistory::image_callback, this);
	if(_image_sub.getNumPublishers() == 0){
		std::stringstream error_msg;
		error_msg << "[FrameHistory] no publishers currently publishing to " << topic_name;
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