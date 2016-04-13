#include <sub8_pcl/torpedo_board.hpp>

Sub8TorpedoBoardDetector::Sub8TorpedoBoardDetector()
try : image_transport(nh), rviz("/visualization/torpedo_board")
{
  ROS_INFO("Initializing Sub8TorpedoBoardDetector");
  std::string img_topic_left = "/stereo/left/image_rect_color";
  std::string img_topic_right = "/stereo/right/image_rect_color";
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
    if(current_image_left.channels() != 3){
      ROS_WARN("The left image topic does not contain a color image. Terminating torpedo board detection.");
      return;
    }

    input_bridge = cv_bridge::toCvCopy(right_most_recent.image_msg_ptr, sensor_msgs::image_encodings::BGR8);
    current_image_right = input_bridge->image;
    right_cam_model.fromCameraInfo(right_most_recent.info_msg_ptr);
    if(current_image_left.channels() != 3){
      ROS_WARN("The right image topic does not contain a color image. Terminating torpedo board detection.");
      return;
    }
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
    resp.pose.header.stamp = ros::Time::now();
    ROS_DEBUG("Unable to detect torpedo board");
    return;
  }
  else{
    ROS_DEBUG("Detected torpedo board");
  }

  std::cout << "left corners:" << std::endl;
  BOOST_FOREACH(cv::Point left_corner, board_corners_left){
    std::cout << left_corner << std::endl;
  }
  std::cout << "right corners:" << std::endl;
  BOOST_FOREACH(cv::Point right_corner, board_corners_right){
    std::cout << right_corner << std::endl;
  }

  // Get fundamental matrix and rotation matrix in orer to triangulate points
  // Procedure described in section 9.2.2 of The Bible
  // P_L represents left projectin matrix
  cv::Matx34d P_L_cv = left_cam_model.fullProjectionMatrix();
  cv::Matx34d P_R_cv = right_cam_model.fullProjectionMatrix();
  cv::Matx33d rot_right_cv = right_cam_model.rotationMatrix();
  cv::Matx33d rot_left_cv = left_cam_model.rotationMatrix(); // angle of rotation is only 0.03 radians, is this even significant?
  cv::Matx33d K_left = left_cam_model.fullIntrinsicMatrix();
  cv::Matx33d K_right = right_cam_model.fullIntrinsicMatrix();
  cv::Matx33d Kinv_left = K_left.inv();
  cv::Matx33d Kinv_right = K_right.inv();
  std::cout << "left intrinsic\n" << K_left << std::endl;
  std::cout << "right intrinsic\n" << K_right << std::endl;
  std::cout << "left rotation\n" << rot_left_cv << std::endl;
  std::cout << "right rotation\n" << rot_right_cv << std::endl;
  std::vector<cv::Point2d> left_corners_norm_img_coords;
  std::vector<cv::Point2d> right_corners_norm_img_coords;
  std::cout << "left corners normalized" << std::endl;
  BOOST_FOREACH(cv::Point2d left_corner, board_corners_left){
    cv::Matx31d left_corner_hom(left_corner.x, left_corner.y, 1);
    cv::Matx31d left_corner_hom_normalized = Kinv_left * left_corner_hom;
    std::cout << left_corner_hom_normalized << std::endl;
    double x = left_corner_hom_normalized(0) / left_corner_hom_normalized(2);
    double y = left_corner_hom_normalized(1) / left_corner_hom_normalized(2);
    left_corners_norm_img_coords.push_back(cv::Point2d(x, y));
  }
  std::cout << "right corners normalized" << std::endl;
  BOOST_FOREACH(cv::Point2d right_corner, board_corners_right){
    cv::Matx31d right_corner_hom(right_corner.x, right_corner.y, 1);
    cv::Matx31d right_corner_hom_normalized = Kinv_right * right_corner_hom;
    std::cout << right_corner_hom_normalized << std::endl;
    double x = right_corner_hom_normalized(0) / right_corner_hom_normalized(2);
    double y = right_corner_hom_normalized(1) / right_corner_hom_normalized(2);
    right_corners_norm_img_coords.push_back(cv::Point2d(x, y));
  }

    // Match corresponding corner coordinates from both images
  std::vector< std::pair<cv::Point2d, cv::Point2d> > corresponding_corners;
  BOOST_FOREACH(cv::Point2d left_corner, left_corners_norm_img_coords){
    double min_distance = std::numeric_limits<double>::infinity();
    int matching_idx = 0;
    for(size_t j = 0; j < right_corners_norm_img_coords.size(); j++){
      cv::Point2d diff = left_corner - right_corners_norm_img_coords[j];
      double distance = sqrt(pow(diff.x, 2.0) + pow(diff.y, 2.0));
      if(distance < min_distance){
        min_distance = distance;
        matching_idx = j;
      }
    }
    corresponding_corners.push_back(std::pair<cv::Point2d, cv::Point2d>(left_corner, right_corners_norm_img_coords[matching_idx]));
    std::cout << "Matching Pair : " << corresponding_corners[corresponding_corners.size() -1].first << " | " << corresponding_corners[corresponding_corners.size() -1].second << std::endl;
  }


  // Convert opencv Matx to Mat
  cv::Mat P_L_cv_mat = cv::Mat(P_L_cv);
  cv::Mat P_R_cv_mat = cv::Mat(P_R_cv);
  cv::Mat rot_right_cv_mat = cv::Mat(rot_right_cv);

  // Take pseudoinverse (had to convert from matx to mat because mat .inv() method only supports square matrices)
  cv::Mat P_L_inv_cv_mat;
  cv::invert(P_L_cv_mat, P_L_inv_cv_mat, cv::DECOMP_SVD);

  // std::cout << "left_pcv\n" <<  P_L_cv_mat << std::endl;
  // std::cout << "right_pcv\n" <<  P_R_cv_mat << std::endl;
  std::cout << "left_pinvcv_mat\n" <<  P_L_inv_cv_mat << std::endl;

  cv::Matx41d C(0, 0, 0, 1);
  cv::Matx31d cv_epipole_im2 = P_R_cv * C;
  cv::Matx33d cv_cross_with_epipole_im2;
  cv_cross_with_epipole_im2 << 0, -cv_epipole_im2(2, 0),  cv_epipole_im2(1, 0),
            cv_epipole_im2(2, 0),                  0,    -cv_epipole_im2(0, 0),
           -cv_epipole_im2(1, 0),  cv_epipole_im2(0, 0),                     0;
  cv::Mat cv_fundamental_matrix = cv::Mat(cv_cross_with_epipole_im2) * cv::Mat(P_R_cv) * P_L_inv_cv_mat;
  cv::Mat essential = cv::Mat(K_right.t()) * cv_fundamental_matrix * cv::Mat(K_left);

  // Map Eigen matrix to opencv Mat data
  Eigen::Map<Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor> > essential_eigen_map(reinterpret_cast<double*>(essential.data), 3 ,3);
  Eigen::Map<Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor> > rot_right_eigen_map(reinterpret_cast<double*>(rot_right_cv_mat.data), 3 ,3);

  // Create eigen matrix from map
  Eigen::MatrixXd essential_eigen = essential_eigen_map;
  Eigen::MatrixXd rot_right = rot_right_eigen_map;

  // std::cout << "left_proj_eig\n" <<  P_L << std::endl;
  // std::cout << "right_proj_eig\n" <<  P_R << std::endl;
  // std::cout << "left_proj_inv_eig\n" <<  P_L_inv << std::endl;

  std::cout << "fundamental_cv\n" << cv_fundamental_matrix << std::endl;

  // Calculate 3d coordinates of corner points
  std::vector<Eigen::Vector3d> corners_3d;
  for(size_t i = 0; i < corresponding_corners.size(); i++){
    cv::Point2d pt_L, pt_R;
    pt_L = corresponding_corners[i].first;
    pt_R = corresponding_corners[i].second;
    Eigen::Vector3d current_corner = sub::triangulate_image_coordinates(pt_L, pt_R, essential_eigen, rot_right);
    corners_3d.push_back(current_corner);
  }

  // Calculate board position (center of board) in 3d
  Eigen::Vector3d position;
  Eigen::Vector3d sum(0,0,0);
  BOOST_FOREACH(Eigen::Vector3d corner, corners_3d){
    sum = sum + corner;
  }
  position = sum / 4.0;

  // Project board center into left image and visualize
  cv::Matx41d position_hom(position(0), position(1), position(2), 1);
  cv::Matx31d centroid_hom = P_L_cv * position_hom;
  cv::Point centroid_img_coords(centroid_hom(0) / centroid_hom(2), centroid_hom(1) / centroid_hom(2));
  cv::circle(current_image_left, centroid_img_coords, 5, cv::Scalar(0, 0, 255), -1);
  std::cout << "centroid_3d_hom\n" << centroid_hom.t() << std::endl;
  std::cout << "centroid_img_coords " << centroid_img_coords << std::endl;
  cv::imshow("centroid projected", current_image_left);

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
  
  Eigen::Vector3f position;
  determine_torpedo_board_position(resp);
  std::string left_tf_frame = left_cam_model.tfFrame();
  rviz.visualize_torpedo_board(resp.pose.pose, left_tf_frame);
  cv::waitKey(1);
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
  bool corners_success = false;
  cv::Mat convex_hull_working_img = segmented_board.clone();
  std::vector<sub::Contour> contours, connected_contours;
  sub::Contour convex_hull, corner_points;

  /// Find contours
  cv::findContours( convex_hull_working_img, contours, CV_RETR_EXTERNAL, 
                    CV_CHAIN_APPROX_SIMPLE);

  // Put longest 2 contours at beginning of "contours" vector
  if(contours.size() > 2){
    std::partial_sort(contours.begin(), contours.begin() + 2, contours.end(), sub::larger_contour);
  }
  if(contours.size() < 2){    // prevent out of range access of contours
    corners_success = false;
    return corners_success;
  }

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
  if(contours.size() > 1){
    std::partial_sort(connected_contours.begin(), connected_contours.begin() + 1,
                                                  connected_contours.end(), sub::larger_contour);
  }

  // Find convex hull of connected panels
  cv::convexHull(connected_contours[0], convex_hull);
  // ROS_INFO((boost::format("convex hull size = %1%") % convex_hull.size()).str().c_str() );
  size_t poly_pts = convex_hull.size();
  const int max_iterations = 50;
  int total_iterations = 0;
  double epsilon = 1;
  double epsilon_step = 0.5;
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
    ROS_DEBUG((boost::format("num_corners=%1% epsilon=%2% iters=%3%") % convex_hull.size() % epsilon % total_iterations).str().c_str() );
  }

  // Scale corner image coordinates back up to original scale
  for(size_t i = 0; i < convex_hull.size(); i++){
    convex_hull[i].x *= (1 / image_proc_scale);
    convex_hull[i].y *= (1 / image_proc_scale);
  }
  corners = convex_hull;
  return corners_success;
}