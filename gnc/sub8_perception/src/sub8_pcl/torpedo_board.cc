#include <sub8_pcl/torpedo_board.hpp>

Sub8TorpedoBoardDetector::Sub8TorpedoBoardDetector()
try : image_transport(nh), rviz("/visualization/torpedo_board")
{
  ROS_INFO("Initializing Sub8TorpedoBoardDetector");
  std::string img_topic_left = "/stereo/left/image_raw";
  std::string img_topic_right = "/stereo/right/image_raw";
  std::string service_name = "/vision/torpedo_board";
  
  left_image_sub = image_transport.subscribeCamera(img_topic_left, 1, &Sub8TorpedoBoardDetector::left_image_callback, this);
  right_image_sub = image_transport.subscribeCamera(img_topic_right, 1, &Sub8TorpedoBoardDetector::right_image_callback, this);
  
  std::string log_msg = 
    (boost::format("Camera subscriptions: left=%1% right=%2%") % img_topic_left % img_topic_right).str();
  ROS_INFO(log_msg.c_str());

  service =
    nh.advertiseService(service_name, &Sub8TorpedoBoardDetector::request_torpedo_board_position, this);
  log_msg = (boost::format("Advertisied Service: %1%") % service_name).str();
  ROS_INFO(log_msg.c_str());
  ROS_INFO("Sub8TorpedoBoardDetector Initialized");
}
catch(const std::exception &e){
  ROS_ERROR("Exception from within Sub8TorpedoBoardDetector constructor initializer list: ");
  ROS_ERROR(e.what());
}


Sub8TorpedoBoardDetector::~Sub8TorpedoBoardDetector(){

}


void Sub8TorpedoBoardDetector::left_image_callback(const sensor_msgs::ImageConstPtr &image_msg_ptr,
                               const sensor_msgs::CameraInfoConstPtr &info_msg_ptr){
  left_most_recent.image_msg_ptr = image_msg_ptr;
  left_most_recent.info_msg_ptr = info_msg_ptr;
}


void Sub8TorpedoBoardDetector::right_image_callback(const sensor_msgs::ImageConstPtr &image_msg_ptr,
                               const sensor_msgs::CameraInfoConstPtr &info_msg_ptr){
  right_most_recent.image_msg_ptr = image_msg_ptr;
  right_most_recent.info_msg_ptr = info_msg_ptr;
}


void Sub8TorpedoBoardDetector::determine_torpedo_board_position(sub8_msgs::VisionRequest::Response &resp){
  cv_bridge::CvImagePtr input_bridge;
  cv::Mat current_image_left, current_image_right, segmented_board_left, segmented_board_right;

  // Get the most recent frames and camera info for both cameras
  try {
    input_bridge = cv_bridge::toCvCopy(left_most_recent.image_msg_ptr, sensor_msgs::image_encodings::BGR8);
    current_image_left = input_bridge->image;
    left_cam_model.fromCameraInfo(left_most_recent.info_msg_ptr);

    input_bridge = cv_bridge::toCvCopy(right_most_recent.image_msg_ptr, sensor_msgs::image_encodings::BGR8);
    current_image_right = input_bridge->image;
    right_cam_model.fromCameraInfo(right_most_recent.info_msg_ptr);
  } catch (cv_bridge::Exception &ex) {
    ROS_ERROR("[torpedo_board] cv_bridge: Failed to convert images");
    return;
  }

  // Segment Board and find image coordinates of board corners
  sub::Contour board_corners_left, board_corners_right;
  segment_board(current_image_left, segmented_board_left);
  segment_board(current_image_right, segmented_board_right);
  bool found_left = find_board_corners(segmented_board_left, board_corners_left);
  bool found_right = find_board_corners(segmented_board_right, board_corners_right);
  if(!found_left || !found_right){
    resp.found = false;
    ROS_INFO("Unable to detect torpedo board");
    return;
  }

  // Match corresponding corner coordinates from both images
  std::vector< std::pair<cv::Point, cv::Point> > corresponding_corners;
  BOOST_FOREACH(cv::Point left_corner, board_corners_left){
    double min_distance = std::numeric_limits<double>::infinity();
    int matching_idx = 0;
    for(size_t j = 0; j < board_corners_right.size(); j++){
      cv::Point diff = left_corner - board_corners_right[j];
      double distance = sqrt(pow(diff.x, 2.0) + pow(diff.y, 2.0));
      if(distance < min_distance){
        min_distance = distance;
        matching_idx = j;
      }
    }
    corresponding_corners.push_back(std::pair<cv::Point, cv::Point>(left_corner, board_corners_right[matching_idx]));
  }

  // Get Fundamental Matrix from full camera projection matrices
  // Procedure described in section 8.2.2 of The Bible
  cv::Matx34d P_L = left_cam_model.fullProjectionMatrix();
  cv::Matx34d P_R = right_cam_model.fullProjectionMatrix();
  cv::Matx41d C(0, 0, 0, 1);
  cv::Matx43d P_L_inv;
  cv::invert(P_L, P_L_inv);
  cv::Matx31d epipole_im2 = P_R * C;
  cv::Matx33d cross_with_epipole_im2( 0, -epipole_im2(2, 0),  epipole_im2(1, 0),
                     epipole_im2(2, 0),                  0, -epipole_im2(0, 0),
                    -epipole_im2(1, 0),  epipole_im2(0, 0),                  0);
  cv::Matx33d fundamental_mat = cross_with_epipole_im2 * ( P_R * P_L_inv);
  cv::Matx33d rot_matx_right = right_cam_model.rotationMatrix();
  Eigen::Matrix3d fundamental_matrix;
  fundamental_matrix << fundamental_mat(0, 0), fundamental_mat(0, 1), fundamental_mat(0, 2),
                        fundamental_mat(1, 0), fundamental_mat(1, 1), fundamental_mat(1, 2),
                        fundamental_mat(2, 0), fundamental_mat(2, 1), fundamental_mat(2, 2);

  // Calculate 3d coordinates of corner points
  Eigen::Matrix3d R;
  R << rot_matx_right(0, 0), rot_matx_right(0, 1), rot_matx_right(0, 2),
       rot_matx_right(1, 0), rot_matx_right(1, 1), rot_matx_right(1, 2),
       rot_matx_right(2, 0), rot_matx_right(2, 1), rot_matx_right(2, 2);
  std::vector<Eigen::Vector3d> corners_3d;
  for(size_t i = 0; i < corresponding_corners.size(); i++){
    cv::Point pt_L, pt_R;
    pt_L = corresponding_corners[i].first;
    pt_R = corresponding_corners[i].second;
    Eigen::Vector3d current_corner = sub::triangulate_image_coordinates(pt_L, pt_R, fundamental_matrix, R);
    corners_3d.push_back(current_corner);
  }

  // Calculate board position (center of board) in 3d
  Eigen::Vector3d position;
  Eigen::Vector3d sum(0,0,0);
  BOOST_FOREACH(Eigen::Vector3d corner, corners_3d){
    sum = sum + corner;
  }
  position = sum / 4.0;

  // Calculate normal vector to torpedo board
  Eigen::Vector3d normal_vector, edge1, edge2;
  edge1 = corners_3d[0] - corners_3d[1];
  edge2 = corners_3d[2] - corners_3d[1];
  normal_vector = edge1.cross(edge2);
  normal_vector = normal_vector.normalized();

  // Calculate Quaternion representing rotation from z-axis to normal vector
  Eigen::Vector3d z_axis(0, 0, 1);
  Eigen::Quaterniond orientation;
  orientation.setFromTwoVectors(z_axis, normal_vector);

  // Fill service response fields
  std::string tf_frame = left_cam_model.tfFrame();
  resp.pose.header.frame_id = tf_frame;
  resp.pose.header.stamp = ros::Time::now();
  tf::pointEigenToMsg(position, resp.pose.pose.position);
  tf::quaternionEigenToMsg(orientation, resp.pose.pose.orientation);
  resp.found = true;
}


bool Sub8TorpedoBoardDetector::request_torpedo_board_position(sub8_msgs::VisionRequest::Request &req,
                                           sub8_msgs::VisionRequest::Response &resp){
  std::string left_tf_frame = left_cam_model.tfFrame();
  Eigen::Vector3f position;
  determine_torpedo_board_position(resp);
  rviz.visualize_torpedo_board(resp.pose.pose, left_tf_frame);
  return true;
}


void Sub8TorpedoBoardDetector::segment_board(const cv::Mat &src, cv::Mat &dest){

  // Preprocessing
  cv::Mat processing_size_image, hsv_image, hue_blurred, sat_blurred;
  std::vector<cv::Mat> hsv_channels;
  cv::resize(src, processing_size_image, cv::Size(0,0), image_proc_scale, 0.5);
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


bool Sub8TorpedoBoardDetector::find_board_corners(const cv::Mat &segmented_board, sub::Contour &corners){
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
  ROS_INFO((boost::format("convex hull size = %1%") % convex_hull.size()).str().c_str() );
  size_t poly_pts = convex_hull.size();
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
    ROS_INFO((boost::format("corners size = %1% epsilon = %2% iters = %3%") % convex_hull.size() % epsilon % total_iterations).str().c_str() );
  }

  // Scale corner image coordinates back up to original scale
  for(size_t i = 0; i < convex_hull.size(); i++){
    convex_hull[i].x *= (1 / image_proc_scale);
    convex_hull[i].y *= (1 / image_proc_scale);
  }
  corners = convex_hull;
  return corners_success;
}