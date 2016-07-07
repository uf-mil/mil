#include <sub8_vision_lib/torpedo_board.hpp>

///////////////////////////////////////////////////////////////////////////////////////////////////
// Class: Sub8ObjectFinder ////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////////////////

Sub8ObjectFinder::Sub8ObjectFinder() 
  try : image_transport(nh),   rviz("/torpedo_board/visualization/detection") {

  std::stringstream log_msg;
  log_msg << "\nInitializing Sub8ObjectFinder:\n";
  int tab_sz = 4;

  // Default parameters
  std::string img_topic_left_default = "/stereo/left/image_rect_color/";
  std::string img_topic_right_default = "/stereo/right/image_rect_color/";
  std::string activation_default = "/torpedo_board/detection_activation_switch";
  std::string dbg_topic_default = "/torpedo_board/dbg_imgs";
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
    left, 1000, &Sub8ObjectFinder::left_image_callback, this);
  right_image_sub = image_transport.subscribeCamera(
    right, 1000, &Sub8ObjectFinder::right_image_callback, this);
  log_msg << std::setw(1 * tab_sz) << "" << "Camera Subscriptions:\x1b[37m\n"
          << std::setw(2 * tab_sz) << "" << "left  = " << left << std::endl
          << std::setw(2 * tab_sz) << "" << "right = " << right << "\x1b[0m\n";

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
      activation, &Sub8ObjectFinder::detection_activation_switch, this);
  log_msg 
          << std::setw(1 * tab_sz) << "" << "Advertised torpedo board detection switch:\n"
          << std::setw(2 * tab_sz) << "" << "\x1b[37m" << activation << "\x1b[0m\n";


  // Start main detector loop
  run_id = 0;
  boost::thread main_loop_thread(boost::bind(&Sub8ObjectFinder::run, this));
  main_loop_thread.detach();
  log_msg << std::setw(1 * tab_sz) << "" << "Running main detector loop in a background thread\n";

  log_msg << "Sub8ObjectFinder Initialized\n";
  ROS_INFO(log_msg.str().c_str());

} catch (const std::exception &e) {
  ROS_ERROR("Exception from within Sub8ObjectFinder constructor "
            "initializer list: ");
  ROS_ERROR(e.what());
}

Sub8ObjectFinder::~Sub8ObjectFinder() {
  ROS_INFO("Killed Torpedo Board Detector");
}

void Sub8ObjectFinder::run() {
  ros::Rate loop_rate(10);  // process images 10 times per second
  while (ros::ok()) {
    if (active)
      determine_torpedo_board_position();
    loop_rate.sleep();
  }
  return;
}

bool Sub8ObjectFinder::detection_activation_switch(
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

void Sub8ObjectFinder::left_image_callback(
    const sensor_msgs::ImageConstPtr &image_msg_ptr,
    const sensor_msgs::CameraInfoConstPtr &info_msg_ptr) {
  left_mtx.lock();
  left_most_recent.image_msg_ptr = image_msg_ptr;
  left_most_recent.info_msg_ptr = info_msg_ptr;
  left_mtx.unlock();
}

void Sub8ObjectFinder::right_image_callback(
    const sensor_msgs::ImageConstPtr &image_msg_ptr,
    const sensor_msgs::CameraInfoConstPtr &info_msg_ptr) {
  right_mtx.lock();
  right_most_recent.image_msg_ptr = image_msg_ptr;
  right_most_recent.info_msg_ptr = info_msg_ptr;
  right_mtx.unlock();
}

void Sub8ObjectFinder::determine_torpedo_board_position() {

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



  // Get camera projection matrices
  cv::Matx34d left_cam_mat = left_cam_model.fullProjectionMatrix();
  cv::Matx34d right_cam_mat = right_cam_model.fullProjectionMatrix();

  

  // ROS dbg_img visualization
  sensor_msgs::ImagePtr dbg_img_msg =
      cv_bridge::CvImage(std_msgs::Header(), "bgr8", debug_image).toImageMsg();
  debug_image_pub.publish(dbg_img_msg);
  ros::spinOnce();
  return;
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
  Sub8ObjectFinder torpedo_board_detector;
  ros::spin();
}
