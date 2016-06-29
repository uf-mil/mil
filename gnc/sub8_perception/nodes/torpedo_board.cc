#include <sub8_vision_lib/torpedo_board.hpp>

///////////////////////////////////////////////////////////////////////////////////////////////////
// Class: Sub8TorpedoBoardDetector ////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////////////////

Sub8TorpedoBoardDetector::Sub8TorpedoBoardDetector() 
  try : image_transport(nh),   rviz("/torpedo_board/visualization/detection") {

  cout << "HELLO!\n";
  std::stringstream log_msg;
  log_msg << "\nInitializing Sub8TorpedoBoardDetector:\n";
  int tab_sz = 4;

  // Default parameters
  std::string img_topic_left_default = "/stereo/left/image_rect_color/";
  std::string img_topic_right_default = "/stereo/right/image_rect_color/";
  std::string activation_default = "/torpedo_board/detection_activation_switch";
  std::string dbg_topic_default = "/torpedo_board/dbg_imgs";
  std::string pose_est_srv_default = "/torpedo_board/pose_est_srv";
  float image_proc_scale_default = 0.5;
  bool generate_dbg_img_default = true;

  // Set image processing scale
  image_proc_scale = ros::param::param<float>("img_proc_scale", image_proc_scale_default);
  log_msg << std::setw(1 * tab_sz) << "" << "Image Processing Scale: \x1b[37m" 
          << image_proc_scale << "\x1b[0m\n";

  // Configure debug image generation
  generate_dbg_img = ros::param::param<bool>("generate_dbg_imgs", generate_dbg_img_default);

  // Subscribe to Cameras (image + camera_info)
  std::string left = ros::param::param<std::string>("input_left", img_topic_left_default);
  std::string right = ros::param::param<std::string>("input_right", img_topic_right_default);
  left_image_sub = image_transport.subscribeCamera(
    left, 1000, &Sub8TorpedoBoardDetector::left_image_callback, this);
  right_image_sub = image_transport.subscribeCamera(
    right, 1000, &Sub8TorpedoBoardDetector::right_image_callback, this);
  log_msg << std::setw(1 * tab_sz) << "" << "Camera Subscriptions:\x1b[37m\n"
          << std::setw(2 * tab_sz) << "" << "left  = " << left << std::endl
          << std::setw(2 * tab_sz) << "" << "right = " << right << "\x1b[0m\n";

  // Register Pose Estimation Service Client
  std::string pose_est_srv = ros::param::param<std::string>("pose_est_srv", pose_est_srv_default);
  pose_client = nh.serviceClient<sub8_msgs::TorpBoardPoseRequest>( pose_est_srv);
  log_msg
      << std::setw(1 * tab_sz) << "" << "Registered as client of the service:\n" 
      << std::setw(2 * tab_sz) << "" << "\x1b[37m" << pose_est_srv << "\x1b[0m\n";

  // Advertise debug image topic
  std::string dbg_topic = ros::param::param<std::string>("dbg_imgs", dbg_topic_default);
  debug_image_pub = image_transport.advertise(dbg_topic, 1, true);
  log_msg << std::setw(1 * tab_sz) << "" << "Advertised debug image topic:\n"
          << std::setw(2 * tab_sz) << "" << "\x1b[37m" << dbg_topic << "\x1b[0m\n";

  // Setup debug image quadrants
  cv::Size input_frame_size(644, 482);  // This needs to be changed if we ever change
                                        // the camera settings for frame size
  cv::Size proc_size(cvRound(image_proc_scale * input_frame_size.width),
                           cvRound(image_proc_scale * input_frame_size.height));
  cv::Size dbg_img_size(proc_size.width * 2, proc_size.height * 2);
  debug_image = cv::Mat(dbg_img_size, CV_8UC3, cv::Scalar(0, 0, 0));
  upper_left = cv::Rect(cv::Point(0, 0), proc_size);
  upper_right = cv::Rect(cv::Point(proc_size.width, 0), proc_size);
  lower_left = cv::Rect(cv::Point(0, proc_size.height), proc_size);
  lower_right = cv::Rect(cv::Point(proc_size.width, proc_size.height), proc_size);

  // Advertise detection activation switch
  active = false;
  std::string activation = ros::param::param<std::string>("activation", activation_default);
  detection_switch = nh.advertiseService(
      activation, &Sub8TorpedoBoardDetector::detection_activation_switch, this);
  log_msg 
          << std::setw(1 * tab_sz) << "" << "Advertised torpedo board detection switch:\n"
          << std::setw(2 * tab_sz) << "" << "\x1b[37m" << activation << "\x1b[0m\n";


  // Start main detector loop
  run_id = 0;
  boost::thread main_loop_thread(boost::bind(&Sub8TorpedoBoardDetector::run, this));
  main_loop_thread.detach();
  log_msg << std::setw(1 * tab_sz) << "" << "Running main detector loop in a background thread\n";

  log_msg << "Sub8TorpedoBoardDetector Initialized\n";
  ROS_INFO(log_msg.str().c_str());

} catch (const std::exception &e) {
  ROS_ERROR("Exception from within Sub8TorpedoBoardDetector constructor "
            "initializer list: ");
  ROS_ERROR(e.what());
}

Sub8TorpedoBoardDetector::~Sub8TorpedoBoardDetector() {
  ROS_INFO("Killed Torpedo Board Detector");
}

void Sub8TorpedoBoardDetector::run() {
  ros::Rate loop_rate(10);  // process images 10 times per second
  while (ros::ok()) {
    if (active)
      determine_torpedo_board_position();
    loop_rate.sleep();
  }
  return;
}

bool Sub8TorpedoBoardDetector::detection_activation_switch(
    sub8_msgs::TBDetectionSwitch::Request &req,
    sub8_msgs::TBDetectionSwitch::Response &resp) {
  resp.success = false;
  std::stringstream ros_log;
  ros_log << "\x1b[1;31mSetting torpedo board detection to: \x1b[1;37m"
          << (req.detection_switch ? "on" : "off") << "\x1b[0m";
  ROS_INFO(ros_log.str().c_str());
  active = req.detection_switch;
  if (active == req.detection_switch) {
    resp.success = true;
    return true;
  }
  return false;
}

void Sub8TorpedoBoardDetector::left_image_callback(
    const sensor_msgs::ImageConstPtr &image_msg_ptr,
    const sensor_msgs::CameraInfoConstPtr &info_msg_ptr) {
  left_mtx.lock();
  left_most_recent.image_msg_ptr = image_msg_ptr;
  left_most_recent.info_msg_ptr = info_msg_ptr;
  left_mtx.unlock();
}

void Sub8TorpedoBoardDetector::right_image_callback(
    const sensor_msgs::ImageConstPtr &image_msg_ptr,
    const sensor_msgs::CameraInfoConstPtr &info_msg_ptr) {
  right_mtx.lock();
  right_most_recent.image_msg_ptr = image_msg_ptr;
  right_most_recent.info_msg_ptr = info_msg_ptr;
  right_mtx.unlock();
}

void Sub8TorpedoBoardDetector::determine_torpedo_board_position() {

  std::stringstream dbg_str;

  // Prevent segfault if service is called before we get valid img_msg_ptr's
  if (left_most_recent.image_msg_ptr == NULL || right_most_recent.image_msg_ptr == NULL) {
    ROS_WARN("Torpedo Board Detector: Image Pointers are NULL.");
    return;
  }

  // Get the most recent frames and camera info for both cameras
  cv_bridge::CvImagePtr input_bridge;
  cv::Mat current_image_left, current_image_right, processing_size_image_left,
      processing_size_image_right, segmented_board_left, segmented_board_right;
  try {

    left_mtx.lock();
    right_mtx.lock();

    // Left Camera
    input_bridge = cv_bridge::toCvCopy(left_most_recent.image_msg_ptr,
                                       sensor_msgs::image_encodings::BGR8);
    current_image_left = input_bridge->image;
    left_cam_model.fromCameraInfo(left_most_recent.info_msg_ptr);
    cv::resize(current_image_left, processing_size_image_left, cv::Size(0, 0),
               image_proc_scale, image_proc_scale);
    if (current_image_left.channels() != 3) {
      ROS_ERROR("The left image topic does not contain a color image.");
      return;
    }

    // Right Camera
    input_bridge = cv_bridge::toCvCopy(right_most_recent.image_msg_ptr,
                                       sensor_msgs::image_encodings::BGR8);
    current_image_right = input_bridge->image;
    right_cam_model.fromCameraInfo(right_most_recent.info_msg_ptr);
    cv::resize(current_image_right, processing_size_image_right, cv::Size(0, 0),
               image_proc_scale, image_proc_scale);
    if (current_image_right.channels() != 3) {
      ROS_ERROR("The right image topic does not contain a color image.");
      return;
    }

    left_mtx.unlock();
    right_mtx.unlock();

  } catch (const std::exception &ex) {
    ROS_ERROR("[torpedo_board] cv_bridge: Failed to convert images");
    left_mtx.unlock();
    right_mtx.unlock();
    return;
  }

  // Enforce approximate image synchronization
  double left_stamp, right_stamp;
  left_stamp = left_most_recent.image_msg_ptr->header.stamp.toSec();
  right_stamp = right_most_recent.image_msg_ptr->header.stamp.toSec();
  if (std::abs(left_stamp - right_stamp) > sync_thresh) {
    ROS_WARN("Left and right images were not sufficiently synchronized");
    debug_image += cv::Scalar(0, 0, 50);
    return;
  }

  // Denoise Images
  /*
    TODO:
    Try inhomogenous diffusion, anisotropic diffusion for better filtering that
    preserves edges
  */
  cv::medianBlur(processing_size_image_left, processing_size_image_left,
                 5); // Magic num: 5 (might need tuning)
  cv::medianBlur(processing_size_image_right, processing_size_image_right, 5);

  if (generate_dbg_img) {
    try {
      processing_size_image_left.copyTo(debug_image(lower_left));
      processing_size_image_right.copyTo(debug_image(lower_right));
    } catch (std::exception &e) {
      std::stringstream error_msg;
      error_msg << "Exception drawing processing frame on debug image quadrant."
                << e.what() << std::endl
                << "processing_size_image_left size: "
                << processing_size_image_left.size()
                << " upper_left size: " << upper_left.size() << std::endl
                << "processing_size_image_right size: "
                << processing_size_image_right.size()
                << " upper_right size: " << upper_right.size() << std::endl
                << "processing img depth:"
                << processing_size_image_left.channels()
                << " dbg_img depth:" << debug_image.channels() << std::endl
                << "processing img type:" << processing_size_image_left.type()
                << " dbg_img type:" << debug_image.type();
      ROS_WARN(error_msg.str().c_str());
    }
  }

  // Segment Board and find image coordinates of board corners
  cv::Mat left_segment_dbg_img, right_segment_dbg_img;
  segment_board(processing_size_image_left, segmented_board_left,
                left_segment_dbg_img);
  segment_board(processing_size_image_right, segmented_board_right,
                right_segment_dbg_img, true);
#ifdef SEGMENTATION_DEBUG
  cv::imshow("segmented right", segmented_board_right);
  cv::waitKey(1);
#endif
  bool found_left =
      find_board_corners(segmented_board_left, left_corners, true);
  bool found_right =
      find_board_corners(segmented_board_right, right_corners, false);
  if (!found_left || !found_right) {
    ROS_DEBUG("Unable to detect torpedo board");
    return;
  } else {
    ROS_DEBUG("Was able to generate 4 corners, more analysis needed to "
              "determine if they belong to the torpedo board.");
  }

  // Color code corner point correspondences on debug image
  if (generate_dbg_img) {
    for (int i = 0; i < 4; i++) {
      cv::Mat ll_dbg = debug_image(lower_left);
      cv::Mat lr_dbg = debug_image(lower_right);
      cv::Point L_img_corner = left_corners[i] * image_proc_scale;
      cv::Point R_img_corner = right_corners[i] * image_proc_scale;
      cv::Scalar color(0, 0, 0);
      switch (i) {
      case 0:
        cv::circle(ll_dbg, L_img_corner, ll_dbg.rows / 50.0, color, -1);
        cv::circle(lr_dbg, R_img_corner, ll_dbg.rows / 50.0, color, -1);
        break;
      case 1:
        color = cv::Scalar(0, 0, 255);
        cv::circle(ll_dbg, L_img_corner, ll_dbg.rows / 50.0, color, -1);
        cv::circle(lr_dbg, R_img_corner, ll_dbg.rows / 50.0, color, -1);
        break;
      case 2:
        color = cv::Scalar(0, 255, 0);
        cv::circle(ll_dbg, L_img_corner, ll_dbg.rows / 50.0, color, -1);
        cv::circle(lr_dbg, R_img_corner, ll_dbg.rows / 50.0, color, -1);
        break;
      case 3:
        color = cv::Scalar(255, 0, 0);
        cv::circle(ll_dbg, L_img_corner, ll_dbg.rows / 50.0, color, -1);
        cv::circle(lr_dbg, R_img_corner, ll_dbg.rows / 50.0, color, -1);
        break;
      }
    }
  }

  // Get camera projection matrices
  cv::Matx34d left_cam_mat = left_cam_model.fullProjectionMatrix();
  cv::Matx34d right_cam_mat = right_cam_model.fullProjectionMatrix();

  // Reconstruct 3d corners from corresponding image points
  std::vector<Eigen::Vector3d> corners_3d;
  Eigen::Vector3d X;
  for (int i = 0; i < 4; i++) {
    cv::Point2d pt_L_ = left_corners[i];
    cv::Point2d pt_R_ = right_corners[i];
    cv::Matx31d pt_L(pt_L_.x, pt_L_.y, 1);
    cv::Matx31d pt_R(pt_R_.x, pt_R_.y, 1);
    cv::Mat X_hom = sub::triangulate_Linear_LS(cv::Mat(left_cam_mat),
                                               cv::Mat(right_cam_mat),
                                               cv::Mat(pt_L), cv::Mat(pt_R));
    X_hom = X_hom / X_hom.at<double>(3, 0);
    X << 
      X_hom.at<double>(0, 0), X_hom.at<double>(1, 0), X_hom.at<double>(2, 0);
    corners_3d.push_back(X);
  }

  // Calculate 3d board position (center of board)
  Eigen::Vector3d position(0, 0, 0);
  dbg_str << "3D Corners:" << std::endl;
  for (int i = 0; i < 4; i++){
    dbg_str << corners_3d[i].transpose() << std::endl;
    position = position + (0.25 * corners_3d[i]);
  }

  // Calculate normal vector to torpedo board (averages normal from two point
  // triples) CRITICAL: normal should point towards the front of the board!
  Eigen::Vector3d normal_vector, prelim_norm_vec1, prelim_norm_vec2, edge1,
      edge2, edge3, edge4;
  edge1 = corners_3d[0] - corners_3d[1];
  edge2 = corners_3d[2] - corners_3d[1];
  edge3 = corners_3d[2] - corners_3d[3];
  edge4 = corners_3d[0] - corners_3d[3];
  prelim_norm_vec1 = edge1.cross(edge2);
  prelim_norm_vec2 = edge3.cross(edge4);
  normal_vector = (prelim_norm_vec1 + prelim_norm_vec2) / 2.0;
  normal_vector = normal_vector.normalized();

  // Calculate the location of the board targets
  Eigen::Vector3d vertical_avg, parallel_avg1, parallel_avg2;
  parallel_avg1 = (edge1 - edge3) / 2.0;
  parallel_avg2 = (edge2 - edge4) / 2.0;
  if (parallel_avg1.squaredNorm() >=
      parallel_avg2.squaredNorm()) { // Vertical edges are longer
    vertical_avg = parallel_avg1;
  } else
    vertical_avg = parallel_avg2;
  if (vertical_avg(1) < 0)
    vertical_avg = -vertical_avg;
  std::vector<Eigen::Vector3d> targets;
  targets.push_back(position + vertical_avg / 4.0);
  targets.push_back(position - vertical_avg / 4.0);

  // Quaternion representing rotation from negative z-axis to normal vector
  Eigen::Vector3d neg_z_axis(0, 0, -1);
  Eigen::Quaterniond orientation;
  orientation.setFromTwoVectors(neg_z_axis, normal_vector);

  /*
    // In case we ever get CERES support on the sub

    // Calculate quaternion encoding only the component of rotation about the
    y-axis (yaw)
    double mag, x, y, z, w;
    x = z = 0;
    y = orientation.y();
    w = orientation.w();
    mag = pow(y * y + w * w, 0.5);
    y /= mag;
    w /= mag;
    Eigen::Quaterniond yaw_quat(w, x, y, z);
    // Declare initial guess for optimization
    double center_x = position(0);
    double center_y = position(1);
    double center_z = position(2);
    double yaw = 2 * acos( yaw_quat.w());
    // Define Problem and add residual block
    ceres::Problem max_likelihood_pose_estimation;
    TorpedoBoardReprojectionCost cost_functor(left_cam_mat, right_cam_mat,
    left_corners, right_corners);
    ceres::CostFunction* reprojection_error_cost = new
    ceres::AutoDiffCostFunction<TorpedoBoardReprojectionCost ,1 ,1, 1, 1,
    1>(&cost_functor);
    max_likelihood_pose_estimation.AddResidualBlock(reprojection_error_cost,
    NULL, &center_x, &center_y, &center_z, &yaw);
    ceres::Solver::Options options;
    options.max_num_iterations = 100;
    options.linear_solver_type = ceres::DENSE_QR;
    ceres::Solve(options, &max_likelihood_pose_estimation, &summary);
  */

  // Fill in TorpBoardPoseRequest (in order)
  sub8_msgs::TorpBoardPoseRequest pose_req;
  pose_req.request.pose_stamped.header.seq = run_id++;
  pose_req.request.pose_stamped.header.stamp.fromSec(
      0.5 * (left_stamp + right_stamp));
  std::string tf_frame = left_cam_model.tfFrame();
  pose_req.request.pose_stamped.header.frame_id = tf_frame;
  tf::pointEigenToMsg(position, pose_req.request.pose_stamped.pose.position);
  tf::quaternionEigenToMsg(orientation,
                           pose_req.request.pose_stamped.pose.orientation);
  for (int i = 0; i < 12; i++) {
    pose_req.request.l_proj_mat[i] = left_cam_mat(i / 4, i % 4);
    pose_req.request.r_proj_mat[i] = right_cam_mat(i / 4, i % 4);
  }
  geometry_msgs::Pose2D corner;
  for (int i = 0; i < 4; i++) {
    corner.x = left_corners[i].x;
    corner.y = left_corners[i].y;
    pose_req.request.l_obs_corners[i] = corner;

    corner.x = right_corners[i].x;
    corner.y = right_corners[i].y;
    pose_req.request.r_obs_corners[i] = corner;
  }

  // Call the pose estimation service
  try{
    if(pose_client.call(pose_req))
      ROS_INFO_THROTTLE(5, "Successfully called pose estimation service");
  }
  catch(std::exception &e){
    ROS_INFO_THROTTLE(5, e.what());
  }

  // Project 3d centroid and targets into both images and visualize
  if (generate_dbg_img) {
    cv::Matx41d position_hom(position(0), position(1), position(2), 1);
    cv::Matx41d target1_hom(targets[0](0), targets[0](1), targets[0](2), 1);
    cv::Matx41d target2_hom(targets[1](0), targets[1](1), targets[1](2), 1);
    cv::Matx31d L_center2d_hom = left_cam_mat * position_hom;
    cv::Matx31d R_center2d_hom = right_cam_mat * position_hom;
    cv::Matx31d L_target1_hom = left_cam_mat * target1_hom;
    cv::Matx31d R_target1_hom = right_cam_mat * target1_hom;
    cv::Matx31d L_target2_hom = left_cam_mat * target2_hom;
    cv::Matx31d R_target2_hom = right_cam_mat * target2_hom;
    cv::Point2d L_center2d(
        L_center2d_hom(0) / L_center2d_hom(2),
        L_center2d_hom(1) / L_center2d_hom(2));
    cv::Point2d R_center2d(
        R_center2d_hom(0) / R_center2d_hom(2),
        R_center2d_hom(1) / R_center2d_hom(2));
    cv::Point2d L_target1(
        L_target1_hom(0) / L_target1_hom(2),
        L_target1_hom(1) / L_target1_hom(2));
    cv::Point2d R_target1(
        R_target1_hom(0) / R_target1_hom(2),
        R_target1_hom(1) / R_target1_hom(2));
    cv::Point2d L_target2(
        L_target2_hom(0) / L_target2_hom(2),
        L_target2_hom(1) / L_target2_hom(2));
    cv::Point2d R_target2(
        R_target2_hom(0) / R_target2_hom(2),
        R_target2_hom(1) / R_target2_hom(2));
    cv::Mat ll_dbg = debug_image(lower_left);
    cv::Mat lr_dbg = debug_image(lower_right);
    cv::Scalar color(255, 0, 255);
    cv::circle(ll_dbg, L_center2d * image_proc_scale, 5, color, -1);
    color = cv::Scalar(255, 0, 255);
    cv::circle(lr_dbg, R_center2d * image_proc_scale, 5, color, -1);
    color = cv::Scalar(255, 255, 0);
    cv::circle(ll_dbg, L_target1 * image_proc_scale, 5, color, -1);
    cv::circle(lr_dbg, R_target1 * image_proc_scale, 5, color, -1);
    cv::circle(ll_dbg, L_target2 * image_proc_scale, 5, color, -1);
    cv::circle(lr_dbg, R_target2 * image_proc_scale, 5, color, -1);
    dbg_str << "centroid_3d: " << position.transpose() << std::endl;
    dbg_str << "centroid_dbg_img_coords_left: "
            << L_center2d *image_proc_scale << std::endl;
    dbg_str << "centroid_dbg_img_coords_right: "
            << R_center2d *image_proc_scale << std::endl;

    std::stringstream left_text, right_text;
    int height = debug_image(lower_left).rows;
    cv::Point header_text_pt(height / 20.0, height / 10.0);
    left_text << "Left  " << left_most_recent.image_msg_ptr->header.stamp;
    right_text << "Right " << right_most_recent.image_msg_ptr->header.stamp;
    int font = cv::FONT_HERSHEY_SIMPLEX;
    double font_scale = 0.0015 * height;
    cv::putText(ll_dbg, left_text.str(), header_text_pt, font,
                font_scale, color);
    cv::putText(lr_dbg, right_text.str(), header_text_pt, font,
                font_scale, color);
  }

  // logs all of the debug statements from the main processing loop
  ROS_DEBUG(dbg_str.str().c_str());

  // Rviz visualization
  rviz.visualize_torpedo_board(pose_req.request.pose_stamped.pose, orientation,
                               targets, corners_3d, tf_frame);

  // ROS dbg_img visualization
  sensor_msgs::ImagePtr dbg_img_msg =
      cv_bridge::CvImage(std_msgs::Header(), "bgr8", debug_image).toImageMsg();
  debug_image_pub.publish(dbg_img_msg);
  ros::spinOnce();
  return;
}

void Sub8TorpedoBoardDetector::segment_board(const cv::Mat &src, cv::Mat &dest,
                                             cv::Mat &dbg_img,
                                             bool draw_dbg_img) {

  // Preprocessing
  cv::Mat hsv_image, hue_segment_dbg_img, sat_segment_dbg_img;
  std::vector<cv::Mat> hsv_channels;
  cv::cvtColor(src, hsv_image, CV_BGR2HSV);
  cv::split(hsv_image, hsv_channels);

  // Histogram parameters
  int hist_size = 256; // bin size
  float range[] = {0, 255};
  const float *ranges[] = {range};

  // Segment out torpedo board
  cv::Mat threshed_hue, threshed_sat, segmented_board;
  int target_yellow = 20;
  int target_saturation = 180;
  sub::statistical_image_segmentation(
      hsv_channels[0], threshed_hue, hue_segment_dbg_img, hist_size, ranges,
      target_yellow, "Hue", draw_dbg_img, 6, 3.0, 3.0);
  sub::statistical_image_segmentation(
      hsv_channels[1], threshed_sat, sat_segment_dbg_img, hist_size, ranges,
      target_saturation, "Saturation", draw_dbg_img, 6, 0.1, 0.1);
#ifdef SEGMENTATION_DEBUG
  cv::imshow("segment board input", src);
  cv::imshow("hue segment", threshed_hue);
  cv::imshow("sat segment", threshed_sat);
  cv::waitKey(1);
#endif
  segmented_board = threshed_hue / 2.0 + threshed_sat / 2.0;
  cv::medianBlur(segmented_board, segmented_board, 5);
  cv::threshold(segmented_board, segmented_board, 255 * 0.9, 255,
                cv::THRESH_BINARY);
  dest = segmented_board;

  if (generate_dbg_img && draw_dbg_img) {
    try {
      hue_segment_dbg_img.copyTo(debug_image(upper_left));
      sat_segment_dbg_img.copyTo(debug_image(upper_right));
    } catch (std::exception &e) {
      std::stringstream error_msg;
      error_msg << "Exception thrown drawing segmentation sbg image on "
                   "debug_image quadrant" << std::endl
                << "hue_segment_dbg_img size: " << hue_segment_dbg_img.size()
                << " upper_left size: " << upper_left.size() << std::endl
                << "sat_segment_dbg_img size: " << sat_segment_dbg_img.size()
                << " upper_right size: " << upper_right.size() << std::endl
                << "processing img depth: " << hue_segment_dbg_img.channels()
                << " dbg_img depth: " << debug_image.channels() << std::endl
                << "processing img type: " << hue_segment_dbg_img.type()
                << " dbg_img type: " << debug_image.type() << std::endl
                << e.what();
      ROS_WARN(error_msg.str().c_str());
    }
  }
}

bool Sub8TorpedoBoardDetector::find_board_corners(
    const cv::Mat &segmented_board, std::vector<cv::Point> &corners, bool draw_dbg_left) {
  bool corners_success = false;
  corners.clear();
  cv::Mat convex_hull_working_img = segmented_board.clone();
  std::vector< std::vector<cv::Point> > contours, connected_contours;
  std::vector<cv::Point> convex_hull, corner_points;

  /// Find contours
  cv::findContours(convex_hull_working_img, contours, CV_RETR_EXTERNAL,
                   CV_CHAIN_APPROX_SIMPLE);

  // Put longest 2 contours at beginning of "contours" vector
  if (contours.size() > 2) {
    std::partial_sort(contours.begin(), contours.begin() + 2, contours.end(),
                      sub::larger_contour);
  }
  if (contours.size() < 2) { // prevent out of range access of contours
    corners_success = false;
    return corners_success;
  }

  // Draw preliminary edge of torpedo_board on debug_image
  if (generate_dbg_img) {
    if (draw_dbg_left) { // Draw on lower left quadrant
      cv::drawContours(debug_image(lower_left), contours, 0,
                       cv::Scalar(255, 255, 255), 2);
      cv::drawContours(debug_image(lower_left), contours, 1,
                       cv::Scalar(255, 255, 255), 2);
    } else { // Draw on lower right quadrant
      cv::drawContours(debug_image(lower_right), contours, 0,
                       cv::Scalar(255, 255, 255), 2);
      cv::drawContours(debug_image(lower_right), contours, 1,
                       cv::Scalar(255, 255, 255), 2);
    }
  }

  // Connect yellow pannels
  cv::Point pt1 = sub::contour_centroid(contours[0]);
  cv::Point pt2 = sub::contour_centroid(contours[1]);
  convex_hull_working_img =
      cv::Mat::zeros(convex_hull_working_img.size(), CV_8UC1);
  cv::drawContours(convex_hull_working_img, contours, 0, cv::Scalar(255));
  cv::drawContours(convex_hull_working_img, contours, 1, cv::Scalar(255));
  cv::line(convex_hull_working_img, pt1, pt2, cv::Scalar(255));
  cv::findContours(convex_hull_working_img, connected_contours,
                   CV_RETR_EXTERNAL, CV_CHAIN_APPROX_SIMPLE);

  // Put longest contour at beginning of "connected_contours" vector
  if (contours.size() > 1) {
    std::partial_sort(connected_contours.begin(),
                      connected_contours.begin() + 1, connected_contours.end(),
                      sub::larger_contour);
  }

  // Find convex hull of connected panels
  cv::convexHull(connected_contours[0], convex_hull);
  size_t poly_pts = convex_hull.size();
  const int max_iterations = 50;
  int total_iterations = 0;
  double epsilon = 1;
  double epsilon_step = 0.5;
  for (int i = 0; i < max_iterations; i++) {
    cv::approxPolyDP(convex_hull, convex_hull, epsilon, true);
    if (convex_hull.size() == poly_pts)
      epsilon += epsilon_step;
    poly_pts = convex_hull.size();
    if (poly_pts == 4) {
      corners_success = true;
      total_iterations = i + 1;
      break;
    }
  }
  if (!corners_success)
    ROS_WARN("Failed to generate the four corners of board from image");
  else {
    std::stringstream ros_log;
    ros_log << "num_corners=" << convex_hull.size() << "epsilon=" << epsilon
            << " iters=" << total_iterations;
    ROS_DEBUG(ros_log.str().c_str());
  }

  // Scale corner image coordinates back up to original scale
  for (size_t i = 0; i < convex_hull.size(); i++) {
    convex_hull[i].x *= (1 / image_proc_scale);
    convex_hull[i].y *= (1 / image_proc_scale);
  }

  // Order corners in this order : {TL, TR, BR, BL}
  float center_x = 0;
  float center_y = 0;
  for(int i = 0; i < 4; i++){
    center_x +=  (0.25 * convex_hull[i].x);
    center_y +=  (0.25 * convex_hull[i].y);
  }
  std::vector<float> convex_hull_angles;
  for(int i = 0; i < 4; i++){
    float x = convex_hull[i].x - center_x;
    float y = -convex_hull[i].y + center_y; // undoes effect of downward y in CV
    convex_hull_angles.push_back(atan2(y, x));
  }
  // Sorting by descending corresponding angle ensures this order
  std::vector<bool> pushed(4, false);
  int largest_angle_idx = 0;
  float largest_angle;
  for(int i = 0; i < 4; i++){
    largest_angle = -M_PI;
    for(int j = 0; j < 4; j++){
      if(!pushed[j] && (convex_hull_angles[j] >= largest_angle)){
        largest_angle_idx = j;
        largest_angle = convex_hull_angles[j];
      }
    }
    corners.push_back(convex_hull[largest_angle_idx]);
    pushed[largest_angle_idx] = true;
  }
  // for(int i = 0; i < 4; i++) std::cout << "\x1b[33m" << corners[i] << std::endl << "\x1b[0m";
  return corners_success;
}

///////////////////////////////////////////////////////////////////////////////////////////////////
// Class: TorpedoBoardReprojectionCost ////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////////////////

TorpedoBoardReprojectionCost::TorpedoBoardReprojectionCost(
    cv::Matx34d &proj_mat_L, cv::Matx34d &proj_mat_R,
    std::vector<cv::Point> &image_corners_L,
    std::vector<cv::Point> &image_corners_R)
    : proj_L(proj_mat_L), proj_R(proj_mat_R), img_corners_L(image_corners_L),
      img_corners_R(image_corners_R) {
  if (image_corners_L.size() != 4 || image_corners_R.size() != 4) {
    throw std::invalid_argument("Corner vectors should contain 4 points.");
  }
}

TorpedoBoardReprojectionCost::~TorpedoBoardReprojectionCost() {
  std::cout << "TorpedoBoardReprojectionCost being destructed." << std::endl;
}

std::vector<cv::Point> TorpedoBoardReprojectionCost::getProjectedCorners(
    double center_x, double center_y, double center_z, double yaw,
    cv::Matx34d &proj_matrix) {

  // Calculate 3d Corners
  std::vector<cv::Matx41d> corners3d;
  double x, y, z;
  x = center_x - 0.5 * width_m * cos(yaw);
  y = center_y - 0.5 * height_m;
  z = center_z - 0.5 * width_m * sin(yaw);
  corners3d.push_back(cv::Matx41d(x, y, z, 1.0));
  x = center_x + 0.5 * width_m * cos(yaw);
  z = center_z + 0.5 * width_m * sin(yaw);
  corners3d.push_back(cv::Matx41d(x, y, z, 1.0));
  y = center_y + 0.5 * height_m;
  corners3d.push_back(cv::Matx41d(x, y, z, 1.0));
  x = center_x - 0.5 * width_m * cos(yaw);
  z = center_z - 0.5 * width_m * sin(yaw);
  corners3d.push_back(cv::Matx41d(x, y, z, 1.0));
  // untested !

  // Project Corners
  std::vector<cv::Point> proj_corners;
  cv::Matx31d proj_hom;
  for (size_t i = 0; i < 4; i++) {
    proj_hom = proj_matrix * corners3d[i];
    proj_corners.push_back(cv::Point(proj_hom(0, 0) / proj_hom(2, 0),
                                     proj_hom(1, 0) / proj_hom(2, 0)));
  }

  return proj_corners;
}

template <typename T>
bool TorpedoBoardReprojectionCost::
operator()(const T *const x, const T *const y, const T *const z,
           const T *const yaw, T *residual) const {
  // Initialize residual in case of garbage within
  residual[0] = 0;

  // Calculate reprojection error from left image
  std::vector<cv::Point2d> model_corners_L =
      getProjectedCorners(x, y, z, yaw, proj_L);

  // Calculate reprojection error from right image
  std::vector<cv::Point2d> model_corners_R =
      getProjectedCorners(x, y, z, yaw, proj_R);

  double squared_distances_L[4][4];
  double squared_distances_R[4][4];

  // Calculate squared distances for all possible point pairs from the 2 sets
  for (int i = 0; i < 4; i++) {
    for (int j = 0; j < 4; j++) {
      squared_distances_L[i][j] =
          pow(model_corners_L[i].x - img_corners_L[j].x, 2.0) +
          pow(model_corners_L[i].y - img_corners_L[j].y, 2.0);
      squared_distances_R[i][j] =
          pow(model_corners_R[i].x - img_corners_R[j].x, 2.0) +
          pow(model_corners_R[i].y - img_corners_R[j].y, 2.0);
    }
  }

  // Add only the smallest squared distance in each of the rows of the above
  // arrays to the residual
  // (smallest implies a point correspondence)
  double least_dist_L, least_dist_R;
  int idx_least_dist_L, idx_least_dist_R;
  for (int i = 0; i < 4; i++) {

    least_dist_L = squared_distances_L[i][0];
    least_dist_R = squared_distances_R[i][0];
    idx_least_dist_L = idx_least_dist_R = 0;

    for (int j = 1; j < 4; j++) {
      if (squared_distances_L[i][j] < least_dist_L) {
        least_dist_L = squared_distances_L[i][j];
        idx_least_dist_L = j;
      }
      if (squared_distances_R[i][j] < least_dist_R) {
        least_dist_R = squared_distances_R[i][j];
        idx_least_dist_R = j;
      }
    }

    residual[0] += least_dist_L;
    residual[0] += least_dist_R;
  }

  return true;
}

///////////////////////////////////////////////////////////////////////////////////////////////////
// Main ///////////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////////////////

int main(int argc, char **argv) {
  ros::init(argc, argv, "torpedo_board_perception");
  ROS_INFO("Initializing node /torpedo_board_perception");
  Sub8TorpedoBoardDetector torpedo_board_detector;
  ros::spin();
}
