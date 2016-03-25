#include <sub8_pcl/torpedo_board.hpp>

Sub8TorpedoBoardDetector::Sub8TorpedoBoardDetector()
try : frame_history("/forward_camera/image_color", 10), image_transport(nh)
{
  std::string img_topic = /*"/forward_camera/image_color"*/ "/stereo/left/image_raw";
  ROS_INFO("Constructing Sub8TorpedoBoardDetector");
  image_sub = image_transport.subscribeCamera(img_topic, 1, &Sub8TorpedoBoardDetector::image_callback, this);
  // ROS_INFO("Subscribed to  %s ", img_topic.c_str());
}
catch(const std::exception &e){
  ROS_ERROR("Error constructing Sub8TorpedoBoardDetector using initializer list: ");
  ROS_ERROR(e.what());
}


Sub8TorpedoBoardDetector::~Sub8TorpedoBoardDetector(){

}


void Sub8TorpedoBoardDetector::image_callback(const sensor_msgs::ImageConstPtr &image_msg,
                               const sensor_msgs::CameraInfoConstPtr &info_msg){
  cv_bridge::CvImagePtr input_bridge;
  cv::Mat current_image, segmented_board_left, segmented_board_right;
  try {
    input_bridge = cv_bridge::toCvCopy(image_msg, sensor_msgs::image_encodings::BGR8);
    current_image = input_bridge->image;
  } catch (cv_bridge::Exception &ex) {
    ROS_ERROR("[torpedo_board] cv_bridge: Failed to convert image");
    return;
  }
  cam_model.fromCameraInfo(info_msg);
  segment_board(current_image, segmented_board_left);

  sub::Contour board_corners;
  find_board_corners(segmented_board_left, board_corners);
  BOOST_FOREACH(cv::Point pt, board_corners){
    cv::circle(segmented_board_left, pt, 5, cv::Scalar(120), -1);
  }
#ifdef VISUALIZE
  cv::imshow("segmented board", segmented_board_left);
#endif

  cv::waitKey(1);
}


void Sub8TorpedoBoardDetector::determine_torpedo_board_position(const image_geometry::PinholeCameraModel &cam_model,
                                             const cv::Mat &image_raw){

}


bool Sub8TorpedoBoardDetector::request_torpedo_board_position(sub8_msgs::VisionRequest::Request &req,
                                           sub8_msgs::VisionRequest::Response &resp){
  return true;
}


void Sub8TorpedoBoardDetector::segment_board(const cv::Mat &src, cv::Mat &dest){

  // Preprocessing
  cv::Mat processing_size_image, hsv_image, hue_blurred, sat_blurred;
  std::vector<cv::Mat> hsv_channels;
  cv::resize(src, processing_size_image, cv::Size(0,0), 0.5, 0.5);
  cv::cvtColor(processing_size_image, hsv_image, CV_BGR2HSV);
  cv::split(hsv_image, hsv_channels);
  cv::medianBlur(hsv_channels[0], hue_blurred, 5); // Magic num: 5 (might need tuning)
  cv::medianBlur(hsv_channels[1], sat_blurred, 5);
  // cv::imshow("hue median filtered", hue_blurred);

  // Histogram parameters
  int hist_size = 256;    // bin size
  float range[] = { 0, 255 };
  const float *ranges[] = { range };

  // Segment out torpedo board
  cv::Mat threshed_hue, threshed_sat, segmented_board;
  int target_yellow = 20;
  int target_saturation = 180;
  sub::statistical_image_segmentation(hue_blurred, threshed_hue, hist_size, ranges,
                                      target_yellow, "Hue", 6, 0.5, 0.5);
  sub::statistical_image_segmentation(sat_blurred, threshed_sat, hist_size, ranges,
                                      target_saturation, "Saturation", 6, 0.1, 0.1);
  segmented_board = threshed_hue / 2.0 + threshed_sat / 2.0;
  cv::threshold(segmented_board, segmented_board, 255*0.75, 255, cv::THRESH_BINARY);
  
#ifdef VISUALIZE
  // cv::imshow("segmented board", segmented_board);
#endif

  // cv::resize(segmented_board, segmented_board, cv::Size(0,0), 2, 2);
  dest = segmented_board;
}


void Sub8TorpedoBoardDetector::find_board_corners(const cv::Mat &segmented_board, sub::Contour &corners){
  cv::Mat convex_hull_working_img = segmented_board.clone();
  std::vector<sub::Contour> contours, connected_contours;
  sub::Contour convex_hull, corner_points;

  /// Find contours
  cv::findContours( convex_hull_working_img, contours, CV_RETR_EXTERNAL, 
                    CV_CHAIN_APPROX_SIMPLE);

  // Put longest 2 contours at beginning of "contours" vector
  std::partial_sort(contours.begin(), contours.begin() + 2, contours.end(), sub::larger_contour);

  // Connect yellow pannels
  cv::Point pt1 = sub::contour_centroid(contours[0]);
  cv::Point pt2 = sub::contour_centroid(contours[1]);
  convex_hull_working_img = cv::Mat::zeros(convex_hull_working_img.size(), CV_8UC1);
  cv::drawContours(convex_hull_working_img, contours, 0, cv::Scalar(255));
  cv::drawContours(convex_hull_working_img, contours, 1, cv::Scalar(255));
  cv::line(convex_hull_working_img, pt1, pt2, cv::Scalar(255));
  cv::findContours( convex_hull_working_img, connected_contours, CV_RETR_EXTERNAL, 
                    CV_CHAIN_APPROX_SIMPLE);

  // Put longest contour at beginning of "connected_contours" vector
  std::partial_sort(connected_contours.begin(), connected_contours.begin() + 1,
                                                connected_contours.end(), sub::larger_contour);

  // Find convex hull of connected panels
  cv::convexHull(connected_contours[0], convex_hull);
  cv::Point board_centroid = sub::contour_centroid(convex_hull);
  ROS_INFO((boost::format("convex hull size = %1%") % convex_hull.size()).str().c_str() );
  int poly_pts = convex_hull.size();
  const int max_iterations = 50;
  int total_iterations = 0;
  double epsilon = 1;
  double epsilon_step = 0.5;
  bool corners_success = false;
  for(int i = 0; i < max_iterations; i++){
    cv::approxPolyDP(convex_hull, convex_hull, epsilon, true);
    if(convex_hull.size() == poly_pts) epsilon += epsilon_step;
    poly_pts = convex_hull.size();
    if(poly_pts == 4){
      corners_success = true;
      total_iterations = i + 1;
      break;
    }
  }
  if(!corners_success) ROS_WARN("Failed to generate the four corners of board from image");
  else{
    ROS_INFO((boost::format("corners size = %1% epsilon = %2% iterations = %3%") % convex_hull.size() % epsilon % total_iterations).str().c_str() );
  }
  corners = convex_hull;
}