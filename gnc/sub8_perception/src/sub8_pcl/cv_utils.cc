#include <sub8_pcl/cv_tools.hpp>

namespace sub {

cv::Point contour_centroid(Contour& contour) {
  cv::Moments m = cv::moments(contour, true);
  cv::Point center(m.m10 / m.m00, m.m01 / m.m00);
  return center;
}


cv::MatND smooth_histogram(const cv::MatND &histogram, size_t filter_kernel_size, float sigma){
	cv::MatND hist = histogram.clone();
	std::vector<float> gauss_kernel = generate_gaussian_kernel_1D(filter_kernel_size, sigma);
	size_t histSize = hist.total();
	size_t offset = (filter_kernel_size - 1) / 2;
	for (size_t i = offset; i < histSize-offset; i++ )   // Convolve histogram values with gaussian kernel
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


std::vector<cv::Point> find_histogram_modes(const cv::MatND &histogram){
	std::vector<cv::Point> local_maxima;
	std::vector<cv::Point> modes_prelim, modes;
	float global_maximum = 0;

	// Locate local maxima and find global maximum
	for(size_t idx = 1; idx < histogram.total() - 1; idx++){
		float current_value = histogram.at<float>(idx);
	    if((histogram.at<float>(idx-1) < current_value) && (histogram.at<float>(idx+1) < current_value)){
	    	local_maxima.push_back(cv::Point(idx, current_value));
	    	if(global_maximum < current_value) global_maximum = current_value;
	    }
	}

	// Exclude local maxima that are too close toogether or below a threshold
	int separation_threshold = 10;
	for(size_t i = 0; i < local_maxima.size() - 1; i++){	// Forwards pass
		cv::Point current = local_maxima[i];
		cv::Point next = local_maxima[i + 1];
		if(current.y > global_maximum * 0.1){	// Only accept local maxima greater than one tenth the global maximum
			if(next.x - current.x >= separation_threshold) modes_prelim.push_back(local_maxima[i]);
	  		else{
	  			if(current.y >= next.y) modes_prelim.push_back(current);
	  			else modes_prelim.push_back(next);
	  			i++;  			
	  		}

	  	}
	}
	for(size_t i = modes_prelim.size() - 1; i > 0; i--){	// Backwards pass
		cv::Point current = modes_prelim[i];
		cv::Point previous = modes_prelim[i - 1];
		if(current.y > global_maximum * 0.1){	// Only accept local maxima greater than one tenth the global maximum
			if(current.x - previous.x >= separation_threshold) modes.push_back(local_maxima[i]);
			else{
				if(current.y >= previous.y) modes.push_back(current);
				else modes.push_back(previous);
				i--;  			
			}
		}
	}
	std::cout << std::endl << std::endl << "Modes: ";
	BOOST_FOREACH(cv::Point pt, local_maxima){
		std::cout << pt.x << ' ';
	}
	return modes;
}


unsigned int select_hist_mode(std::vector<cv::Point> &histogram_modes, int target){
  std::vector<int> distances;
  std::cout << std::endl << "Distances: ";
  BOOST_FOREACH(cv::Point mode, histogram_modes){
    distances.push_back(mode.x - target);
    std::cout << mode.x - target << ' ';
  }
  int min_idx = 0;
  for(int i = 0; i < distances.size(); i++){
    if(std::abs(distances[i]) <= std::abs(distances[min_idx])) min_idx = i;
  }
  std::cout << std::endl << "Target: " << target << " Mode Selected: " << histogram_modes[min_idx].x;
  return histogram_modes[min_idx].x;
}


void statistical_image_segmentation(const cv::Mat &src, cv::Mat &dest, const int hist_size,
        							const float** ranges, const int target, std::string image_name, 
        							const float sigma, const float low_thresh_gain, const float high_thresh_gain)
{
	// ROS_INFO("Segmenting Image");
	// Calculate histogram
	cv::MatND hist, hist_smooth, hist_derivative;
	cv::calcHist( &src, 1, 0, cv::Mat(), hist, 1, &hist_size, ranges, true, false );

	// Smooth histogram
	const int kernel_size = 7;
	hist_smooth = sub::smooth_histogram(hist, kernel_size, sigma);

	// Calculate histogram derivative (central finite difference)
	hist_derivative = hist_smooth.clone();
	hist_derivative.at<float>(0) = 0;
	hist_derivative.at<float>(hist_size - 1) = 0;
	for (int i = 1; i < hist_size - 1; ++i)
	{
	hist_derivative.at<float>(i) = (hist_smooth.at<float>(i + 1) - hist_smooth.at<float>(i - 1)) / 2.0;
	}

	// Find target mode
	std::vector<cv::Point> histogram_modes = sub::find_histogram_modes(hist_smooth);
	int target_mode = sub::select_hist_mode(histogram_modes, target);

	// Calculate std dev of histogram slopes
	cv::Scalar hist_deriv_mean, hist_deriv_stddev;
	cv::meanStdDev(hist_derivative, hist_deriv_mean, hist_deriv_stddev);

	// Determine thresholds for cv::inRange() using the std dev of histogram slopes times a gain as a cutoff heuristic
	int high_thresh = target_mode; int low_thresh = target_mode;
	for(int i = target_mode + 1; i < hist_size; i++){
		if(hist_smooth.at<float>(i) - hist_smooth.at<float>(i + 1) < hist_deriv_stddev[0] * high_thresh_gain){
			high_thresh = i;
			break;
		}
	}
	for(int i = target_mode - 1; i > 0; i--){
		if(hist_smooth.at<float>(i) - hist_smooth.at<float>(i - 1) < hist_deriv_stddev[0] * low_thresh_gain){ 
			low_thresh = i; 
			break; 
		}
	}
	// ROS_INFO("%s : THRESHOLDS: low= %d high= %d", image_name.c_str(), low_thresh, high_thresh);

	// Threshold image
	cv::inRange(src, low_thresh, high_thresh, dest);

	#ifdef VISUALIZE
	// Prepare to draw graph of histogram and derivative
	int hist_w = 512; int hist_h = 400;
	int bin_w = cvRound( (double) hist_w/hist_size );
	cv::Mat histImage( hist_h, hist_w, CV_8UC1, cv::Scalar( 0,0,0) );
	cv::Mat histDerivImage( hist_h, hist_w, CV_8UC1, cv::Scalar( 0,0,0) );
	cv::normalize(hist_smooth, hist_smooth, 0, histImage.rows, cv::NORM_MINMAX, -1, cv::Mat() );
	cv::normalize(hist_derivative, hist_derivative, 0, histImage.rows, cv::NORM_MINMAX, -1, cv::Mat() );

	// Draw Graphs
	for( int i = 1; i < hist_size; i++ )
	{
	// Plot image histogram
	cv::line( histImage, cv::Point( bin_w*(i-1), hist_h - cvRound(hist_smooth.at<float>(i-1)) ) ,
	                 cv::Point( bin_w*(i), hist_h - cvRound(hist_smooth.at<float>(i)) ),
	                 cv::Scalar( 255, 0, 0), 2, 8, 0  );
	// Plot image histogram derivative
	cv::line( histDerivImage, cv::Point( bin_w*(i-1), hist_h - cvRound(hist_derivative.at<float>(i-1)) ) ,
	                 cv::Point( bin_w*(i), hist_h - cvRound(hist_derivative.at<float>(i)) ),
	                 cv::Scalar( 122, 0, 0), 1, 8, 0  );
	}

	// Shade in area being segmented under histogram curve
	cv::line( histImage, cv::Point( bin_w*low_thresh, hist_h - cvRound(hist_smooth.at<float>(low_thresh)) ) ,
	                 cv::Point( bin_w*low_thresh, hist_h), cv::Scalar( 125, 0, 0),2);
	cv::line( histImage, cv::Point( bin_w*high_thresh, hist_h - cvRound(hist_smooth.at<float>(high_thresh)) ) ,
                 cv::Point( bin_w*high_thresh, hist_h), cv::Scalar( 125, 0, 0),2);
	cv::floodFill(histImage, cv::Point(bin_w*cvRound(float(low_thresh + high_thresh) / 2.0), hist_h - 1), cv::Scalar(125));

	// Combine graphs into one image and display results
	cv::Mat zeros_like_hists = cv::Mat::zeros(histImage.size(), CV_8UC1);
	std::vector<cv::Mat> hist_graphs_vec;
	hist_graphs_vec.push_back(histImage); 
	hist_graphs_vec.push_back(histDerivImage);
	hist_graphs_vec.push_back(zeros_like_hists);
	cv::Mat hist_graphs;
	cv::merge(hist_graphs_vec, hist_graphs);
	cv::imshow( image_name + " Histogram and Derivative", hist_graphs );

	// Display segmented image
	cv::imshow( image_name + " Statistical Image Segmentation" , dest);
	#endif
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