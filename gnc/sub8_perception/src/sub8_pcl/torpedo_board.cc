#include <sub8_pcl/torpedo_board.hpp>

Sub8TorpedoBoardDetector::Sub8TorpedoBoardDetector(double im_proc_scale, bool gen_dbg_img,
                                                   std::string l_img_topic, std::string r_img_topic,
                                                   std::string srv_name, std::string viz_topic)
try : image_transport(nh), rviz(viz_topic.empty()? "/visualization/torpedo_board" : viz_topic),
      generate_dbg_img(gen_dbg_img)
{
  ROS_INFO("Initializing Sub8TorpedoBoardDetector");

  // Default topic and service names
  std::string img_topic_left = l_img_topic.empty()? "/stereo/left/image_rect_color" : l_img_topic;
  std::string img_topic_right = r_img_topic.empty()? "/stereo/right/image_rect_color" : r_img_topic;
  std::string service_name = srv_name.empty()? "/vision/torpedo_board" : srv_name;
  std::string visualization_topic = viz_topic.empty()? "/visualization/torpedo_board" : viz_topic;
  
  // Subscribe to Cameras (image + camera_info)
  left_image_sub = image_transport.subscribeCamera(img_topic_left, 1, &Sub8TorpedoBoardDetector::left_image_callback, this);
  right_image_sub = image_transport.subscribeCamera(img_topic_right, 1, &Sub8TorpedoBoardDetector::right_image_callback, this);
  std::stringstream log_msg;
  log_msg << "Torpedo Board Detector camera subscriptions:" << std::left << std::endl
          << std::setw(7) << "" << std::setw(8) << " left  = " << img_topic_left << std::endl 
          << std::setw(7) << "" << std::setw(8) << " right = " << img_topic_right;
  ROS_INFO(log_msg.str().c_str());

  // Advertise Detector Service
  service = nh.advertiseService(service_name, &Sub8TorpedoBoardDetector::request_torpedo_board_position, this);
  log_msg.str(""); 
  log_msg.clear();
  log_msg << "Advertisied Service: " << service_name;
  ROS_INFO(log_msg.str().c_str());
  ROS_INFO("Sub8TorpedoBoardDetector Initialized");

  // Advertize debug image topic
  debug_image_pub = image_transport.advertise("dbg_imgs/torpedo_board", 1 , true);

  // Set image processing scale
  image_proc_scale =  im_proc_scale == 0? 0.5 : im_proc_scale;
  log_msg.str(""); 
  log_msg.clear();
  log_msg << "Image Processing Scale: " << image_proc_scale;
  ROS_INFO(log_msg.str().c_str());

  cv::Size input_frame_size(644, 482); // This needs to be changed if we ever change the camera settings for frame size
  cv::Size processing_size(cvRound(image_proc_scale * input_frame_size.width), cvRound(image_proc_scale * input_frame_size.height));
  cv::Size dbg_img_size(processing_size.width * 2, processing_size.height * 2);
  debug_image = cv::Mat(dbg_img_size, CV_8UC3, cv::Scalar(0, 0, 0));
  upper_left = cv::Rect(cv::Point(0, 0), processing_size);
  upper_right = cv::Rect(cv::Point(processing_size.width, 0), processing_size);
  lower_left = cv::Rect(cv::Point(0, processing_size.height), processing_size);
  lower_right = cv::Rect(cv::Point(processing_size.width, processing_size.height), processing_size);

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
  cv::Mat current_image_left, current_image_right, processing_size_image_left, processing_size_image_right, segmented_board_left, segmented_board_right;

  // Get the most recent frames and camera info for both cameras
  try {
    input_bridge = cv_bridge::toCvCopy(left_most_recent.image_msg_ptr, sensor_msgs::image_encodings::BGR8);
    current_image_left = input_bridge->image;
    left_cam_model.fromCameraInfo(left_most_recent.info_msg_ptr);
    cv::resize(current_image_left, processing_size_image_left, cv::Size(0,0), image_proc_scale, image_proc_scale);
    if(current_image_left.channels() != 3){
      ROS_WARN("The left image topic does not contain a color image. Terminating torpedo board detection.");
      return;
    }

    input_bridge = cv_bridge::toCvCopy(right_most_recent.image_msg_ptr, sensor_msgs::image_encodings::BGR8);
    current_image_right = input_bridge->image;
    right_cam_model.fromCameraInfo(right_most_recent.info_msg_ptr);
    cv::resize(current_image_right, processing_size_image_right, cv::Size(0,0), image_proc_scale, image_proc_scale);
    if(current_image_right.channels() != 3){
      ROS_WARN("The right image topic does not contain a color image. Terminating torpedo board detection.");
      return;
    }
  } catch (cv_bridge::Exception &ex) {
    ROS_ERROR("[torpedo_board] cv_bridge: Failed to convert images");
    return;
  }


  if(generate_dbg_img){
    std::cout << "processing img depth:" << processing_size_image_left.channels() << "dbg_img depth:" << debug_image.channels() << std::endl;
    std::cout << "processing img type:" << processing_size_image_left.type() << " dbg_img type:" << debug_image.type() << std::endl;
    try{
      processing_size_image_left.copyTo(debug_image(lower_left));
      processing_size_image_right.copyTo(debug_image(lower_right));
    }
    catch(std::exception& e){
      std::cout << "Exception thrown from block starting at torpedo_board.cc:108\n" << e.what() << std::endl;
    }
  }

  // Segment Board and find image coordinates of board corners
  cv::Mat left_segment_dbg_img, right_segment_dbg_img;
  sub::Contour board_corners_left, board_corners_right;
  segment_board(processing_size_image_left, segmented_board_left, left_segment_dbg_img, true);
  segment_board(processing_size_image_right, segmented_board_right, right_segment_dbg_img);
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

  // Get camera projection matrices
  cv::Matx34d P_L_cv = left_cam_model.fullProjectionMatrix();
  cv::Matx34d P_R_cv = right_cam_model.fullProjectionMatrix();
  cv::Matx33d rot_right_cv = right_cam_model.rotationMatrix();    // angle of rotation is only 0.03 radians, is this even significant?

  // Convert corner coordinates to normalized image coordinates
  cv::Matx33d K_left = left_cam_model.fullIntrinsicMatrix();
  cv::Matx33d K_right = right_cam_model.fullIntrinsicMatrix();
  cv::Matx33d Kinv_left = K_left.inv();
  cv::Matx33d Kinv_right = K_right.inv();
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

   // Get fundamental matrix, derivation in section 9.2.2 of The Bible
  cv::Mat P_L_cv_mat = cv::Mat(P_L_cv);
  cv::Mat rot_right_cv_mat = cv::Mat(rot_right_cv);
  cv::Mat P_L_inv_cv_mat;
  cv::invert(P_L_cv_mat, P_L_inv_cv_mat, cv::DECOMP_SVD);
  cv::Matx41d C(0, 0, 0, 1);
  cv::Matx31d cv_epipole_im2 = P_R_cv * C;
  cv::Matx33d cv_cross_with_epipole_im2;
  cv_cross_with_epipole_im2 << 0, -cv_epipole_im2(2, 0),  cv_epipole_im2(1, 0),
            cv_epipole_im2(2, 0),                  0,    -cv_epipole_im2(0, 0),
           -cv_epipole_im2(1, 0),  cv_epipole_im2(0, 0),                     0;
  cv::Mat cv_fundamental_matrix = cv::Mat(cv_cross_with_epipole_im2) * cv::Mat(P_R_cv) * P_L_inv_cv_mat;

  // Essential matrix from fundamental matrix, reference: CV Bible section 9.6 (formula 9.12)
  cv::Mat essential = cv::Mat(K_right.t()) * cv_fundamental_matrix * cv::Mat(K_left);

  // Convert essential and rotation OpenCV matrices to Eigen matrices
  Eigen::Map<Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor> > essential_eigen_map(reinterpret_cast<double*>(essential.data), 3 ,3);
  Eigen::Map<Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor> > rot_right_eigen_map(reinterpret_cast<double*>(rot_right_cv_mat.data), 3 ,3);
  Eigen::MatrixXd essential_eigen = essential_eigen_map;
  Eigen::MatrixXd rot_right = rot_right_eigen_map;

  // Reconstruct 3d corners from corresponding image points
  std::vector<Eigen::Vector3d> corners_3d;
  for(size_t i = 0; i < corresponding_corners.size(); i++){
    cv::Point2d pt_L, pt_R;
    pt_L = corresponding_corners[i].first;
    pt_R = corresponding_corners[i].second;
    Eigen::Vector3d current_corner = sub::triangulate_image_coordinates(pt_L, pt_R, essential_eigen, rot_right);
    corners_3d.push_back(current_corner);
  }

  // Calculate 3d board position (center of board)
  Eigen::Vector3d position;
  Eigen::Vector3d sum(0,0,0);
  BOOST_FOREACH(Eigen::Vector3d corner, corners_3d){
    sum = sum + corner;
  }
  position = sum / 4.0;

  // Project board center into left image and visualize
  cv::Matx41d position_hom(position(0), position(1), position(2), 1);
  cv::Matx31d centroid_hom = P_L_cv * position_hom;
  cv::Point2d centroid_img_coords(centroid_hom(0) / centroid_hom(2), centroid_hom(1) / centroid_hom(2));
  cv::circle(processing_size_image_left, cv::Point(centroid_img_coords.x / image_proc_scale, 
             centroid_img_coords.x / image_proc_scale), 3, cv::Scalar(0, 0, 255), -1);
  // std::cout << "centroid_3d_hom\n" << centroid_hom.t() << std::endl;
  std::cout << "centroid_img_coords " << centroid_img_coords << std::endl;
  // cv::imshow("left image with visualization", processing_size_image_left);

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
  // std::string tf_frame = left_cam_model.tfFrame();
  // resp.pose.header.frame_id = tf_frame;
  resp.pose.header.stamp = ros::Time::now();
  tf::pointEigenToMsg(position, resp.pose.pose.position);
  tf::quaternionEigenToMsg(orientation, resp.pose.pose.orientation);
  resp.found = true;
}


bool Sub8TorpedoBoardDetector::request_torpedo_board_position(sub8_msgs::VisionRequest::Request &req, 
                                                              sub8_msgs::VisionRequest::Response &resp){
  // Prevent segfault if service is called before we get valid img_msg_ptr's
  if (left_most_recent.image_msg_ptr == NULL || right_most_recent.image_msg_ptr == NULL) {;
    ROS_ERROR("Torpedo Board Detector: Image Pointers are NULL. Unable to continue.");
    return false;
  }
  else{
    // Eigen::Vector3f position;
    determine_torpedo_board_position(resp);
    std::string left_tf_frame = left_cam_model.tfFrame();
    rviz.visualize_torpedo_board(resp.pose.pose, left_tf_frame);
    sensor_msgs::ImagePtr dbg_img_msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", debug_image).toImageMsg();
    debug_image_pub.publish(dbg_img_msg);
    ros::spinOnce();
    // cv::imshow("Debug Img", debug_image); 
    // cv::waitKey(1);
    return true;
  }

}


void Sub8TorpedoBoardDetector::segment_board(const cv::Mat &src, cv::Mat &dest, cv::Mat &dbg_img, bool draw_dbg_img){

  // Preprocessing
  cv::Mat hsv_image, hue_blurred, sat_blurred, hue_segment_dbg_img, sat_segment_dbg_img;
  std::vector<cv::Mat> hsv_channels;
  cv::cvtColor(src, hsv_image, CV_BGR2HSV);
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
  sub::statistical_image_segmentation(hue_blurred, threshed_hue, hue_segment_dbg_img, hist_size, ranges,
                                      target_yellow, "Hue", draw_dbg_img, 6, 0.5, 0.5);
  sub::statistical_image_segmentation(sat_blurred, threshed_sat, sat_segment_dbg_img, hist_size, ranges,
                                      target_saturation, "Saturation", draw_dbg_img, 6, 0.1, 0.1);
  segmented_board = threshed_hue / 2.0 + threshed_sat / 2.0;
  cv::threshold(segmented_board, segmented_board, 255*0.75, 255, cv::THRESH_BINARY);
  dest = segmented_board;
  
  if(generate_dbg_img && draw_dbg_img){
    std::cout << "processing img depth: " << hue_segment_dbg_img.channels() << " dbg_img depth: " << debug_image.channels() << std::endl;
    std::cout << "processing img type: " << hue_segment_dbg_img.type() << " dbg_img type: " << debug_image.type() << std::endl;
    try{
      hue_segment_dbg_img.copyTo(debug_image(upper_left));
      sat_segment_dbg_img.copyTo(debug_image(upper_right));
    }
    catch(std::exception& e){
      std::stringstream dbg_str;
      dbg_str << "hue_segment_dbg_img size: " << hue_segment_dbg_img.size() << " upper_left size: " << upper_left.size() << std::endl;
      dbg_str << "sat_segment_dbg_img size: " << sat_segment_dbg_img.size() << " upper_right size: " << upper_right.size() << std::endl;
      dbg_str << "debug_img size: " << debug_image.size() << std::endl;
      std::cout << dbg_str.str() << "Error from torpedo_board.cc block starting at line 313:\n" << e.what() << std::endl;
    }

  }
    
#ifdef VISUALIZE
  // cv::imshow("segmented board", segmented_board);
#endif
}


bool Sub8TorpedoBoardDetector::find_board_corners(const cv::Mat &segmented_board, sub::Contour &corners, bool draw_dbg_img){
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

    if(draw_dbg_img){
      // cv::Mat ll_mat = debug_image(lower_left);
      // cv::Mat lr_mat = debug_image(lower_right);
      // cv::drawContours(ll_mat, contours, -1, cv::Scalar(255, 255, 255), 2);
      // cv::drawContours(lr_mat, contours, -1, cv::Scalar(255, 255, 255), 2);
    }
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
    std::stringstream ros_log;
    ros_log << "num_corners=" << convex_hull.size() <<  "epsilon=" << epsilon << " iters=" << total_iterations;
    ROS_DEBUG(ros_log.str().c_str());
  }

  // Scale corner image coordinates back up to original scale
  for(size_t i = 0; i < convex_hull.size(); i++){
    convex_hull[i].x *= (1 / image_proc_scale);
    convex_hull[i].y *= (1 / image_proc_scale);
  }
  corners = convex_hull;
  return corners_success;
}