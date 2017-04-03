#include <sub8_vision_lib/torpedo_board.hpp>

using namespace std;
using namespace cv;

///////////////////////////////////////////////////////////////////////////////////////////////////
// Class: Sub8TorpedoBoardDetector ////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////////////////

Sub8TorpedoBoardDetector::Sub8TorpedoBoardDetector() 
  try : image_transport(nh),   rviz("/torpedo_board/visualization/detection") {

  using ros::param::param;

  stringstream log_msg;
  log_msg << "\nInitializing Sub8TorpedoBoardDetector:\n";
  int tab_sz = 4;

  // Default parameters
  string img_topic_left_default = "/stereo/left/image_rect_color/";
  string img_topic_right_default = "/stereo/right/image_rect_color/";
  string activation_default = "/torpedo_board/detection_activation_switch";
  string dbg_topic_default = "/torpedo_board/dbg_imgs";
  string pose_est_srv_default = "/torpedo_board/pose_est_srv";
  float image_proc_scale_default = 0.5;
  int diffusion_time_default = 20;
  int max_features_default = 20;
  int feature_block_size_default = 11;
  float feature_min_distance_default = 20.0;
  bool generate_dbg_img_default = true;
  int frame_height_default = 644;
  int frame_width_default = 482;

  // Set image processing scale
  image_proc_scale = param<float>("/torpedo_vision/img_proc_scale", image_proc_scale_default);
  log_msg << setw(1 * tab_sz) << "" << "Image Processing Scale: \x1b[37m" 
          << image_proc_scale << "\x1b[0m\n";

  // Set diffusion duration in pseudotime
  diffusion_time = param<int>("/torpedo_vision/diffusion_time", diffusion_time_default);
  log_msg << setw(1 * tab_sz) << "" << "Anisotropic Diffusion Duration: \x1b[37m" 
          << diffusion_time << "\x1b[0m\n";

  // Set feature extraction parameters
  max_features = param<int>("/torpedo_vision/max_features", max_features_default);
  log_msg << setw(1 * tab_sz) << "" << "Maximum features: \x1b[37m" 
          << max_features << "\x1b[0m\n";
  feature_block_size = param<int>("/torpedo_vision/feature_block_size",
                                  feature_block_size_default);
  log_msg << setw(1 * tab_sz) << "" << "Feature Block Size: \x1b[37m" 
          << feature_block_size << "\x1b[0m\n";
  feature_min_distance = param<float>("/torpedo_vision/feature_min_distance",
                                    feature_min_distance_default);
  log_msg << setw(1 * tab_sz) << "" << "Feature Minimum Distance: \x1b[37m" 
          << feature_min_distance << "\x1b[0m\n";

  // Configure debug image generation
  generate_dbg_img = param<bool>("/torpedo_vision/generate_dbg_imgs", generate_dbg_img_default);

  // Subscribe to Cameras (image + camera_info)
  string left = param<string>("/torpedo_vision/input_left", img_topic_left_default);
  string right = param<string>("/torpedo_vision/input_right", img_topic_right_default);
  left_image_sub = image_transport.subscribeCamera(
    left, 10, &Sub8TorpedoBoardDetector::left_image_callback, this);
  right_image_sub = image_transport.subscribeCamera(
    right, 10, &Sub8TorpedoBoardDetector::right_image_callback, this);
  log_msg << setw(1 * tab_sz) << "" << "Camera Subscriptions:\x1b[37m\n"
          << setw(2 * tab_sz) << "" << "left  = " << left << endl
          << setw(2 * tab_sz) << "" << "right = " << right << "\x1b[0m\n";

  // Register Pose Estimation Service Client
  string pose_est_srv = param<string>("/torpedo_vision/pose_est_srv", pose_est_srv_default);
  pose_client = nh.serviceClient<sub8_msgs::TorpBoardPoseRequest>( pose_est_srv);
  log_msg
      << setw(1 * tab_sz) << "" << "Registered as client of the service:\n" 
      << setw(2 * tab_sz) << "" << "\x1b[37m" << pose_est_srv << "\x1b[0m\n";

  // Advertise debug image topic
  string dbg_topic = param<string>("/torpedo_vision/dbg_imgs", dbg_topic_default);
  debug_image_pub = image_transport.advertise(dbg_topic, 1, true);
  log_msg << setw(1 * tab_sz) << "" << "Advertised debug image topic:\n"
          << setw(2 * tab_sz) << "" << "\x1b[37m" << dbg_topic << "\x1b[0m\n";

  // Setup debug image quadrants
  int frame_height = param<int>("/torpedo_vision/frame_height", frame_height_default);
  int frame_width = param<int>("/torpedo_vision/frame_width", frame_width_default);
  Size input_frame_size(frame_height, frame_width);  // This needs to be changed if we ever change
                                                         // the camera settings for frame size
  Size proc_size(cvRound(image_proc_scale * input_frame_size.width),
                           cvRound(image_proc_scale * input_frame_size.height));
  Size dbg_img_size(proc_size.width * 2, proc_size.height * 2);
  debug_image = Mat(dbg_img_size, CV_8UC3, Scalar(0, 0, 0));
  upper_left = Rect(Point(0, 0), proc_size);
  upper_right = Rect(Point(proc_size.width, 0), proc_size);
  lower_left = Rect(Point(0, proc_size.height), proc_size);
  lower_right = Rect(Point(proc_size.width, proc_size.height), proc_size);

  // Advertise detection activation switch
  active = false;
  string activation = param<string>("/torpedo_vision/activation", activation_default);
  detection_switch = nh.advertiseService(
      activation, &Sub8TorpedoBoardDetector::detection_activation_switch, this);
  log_msg 
          << setw(1 * tab_sz) << "" << "Advertised torpedo board detection switch:\n"
          << setw(2 * tab_sz) << "" << "\x1b[37m" << activation << "\x1b[0m\n";

  // Start main detector loop
  run_id = 0;
  boost::thread main_loop_thread(boost::bind(&Sub8TorpedoBoardDetector::run, this));
  main_loop_thread.detach();
  log_msg << setw(1 * tab_sz) << "" << "Running main detector loop in a background thread\n";

  log_msg << "Sub8TorpedoBoardDetector Initialized\n";
  ROS_INFO(log_msg.str().c_str());

} catch (const exception &e) {
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
  stringstream ros_log;
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

  stringstream dbg_str;

  // Prevent segfault if service is called before we get valid img_msg_ptr's
  if (left_most_recent.image_msg_ptr == NULL || right_most_recent.image_msg_ptr == NULL) {
    ROS_WARN("Torpedo Board Detector: Image Pointers are NULL.");
    return;
  }

  // Get the most recent frames and camera info for both cameras
  cv_bridge::CvImagePtr input_bridge;
  Mat current_image_left, current_image_right, processing_size_image_left,
      processing_size_image_right, segmented_board_left, segmented_board_right;
  try {

    left_mtx.lock();
    right_mtx.lock();

    // Left Camera
    input_bridge = cv_bridge::toCvCopy(left_most_recent.image_msg_ptr,
                                       sensor_msgs::image_encodings::BGR8);
    current_image_left = input_bridge->image;
    left_cam_model.fromCameraInfo(left_most_recent.info_msg_ptr);
    resize(current_image_left, processing_size_image_left, Size(0, 0),
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
    resize(current_image_right, processing_size_image_right, Size(0, 0),
               image_proc_scale, image_proc_scale);
    if (current_image_right.channels() != 3) {
      ROS_ERROR("The right image topic does not contain a color image.");
      return;
    }

    left_mtx.unlock();
    right_mtx.unlock();

  } catch (const exception &ex) {
    ROS_ERROR("[torpedo_board] cv_bridge: Failed to convert images");
    left_mtx.unlock();
    right_mtx.unlock();
    return;
  }

  // Enforce approximate image synchronization
  double left_stamp, right_stamp;
  left_stamp = left_most_recent.image_msg_ptr->header.stamp.toSec();
  right_stamp = right_most_recent.image_msg_ptr->header.stamp.toSec();
  double sync_error = fabs(left_stamp - right_stamp);
  stringstream sync_msg;
  sync_msg << "Left and right images were not sufficiently synchronized"
           << "\nsync error: " << sync_error << "s";
  if (sync_error > sync_thresh) {
    ROS_WARN(sync_msg.str().c_str());
    debug_image += Scalar(0, 0, 50);
    return;
  }

  // Denoise Images
  Mat diffusion_size_left, diffusion_size_right;
  resize(current_image_left, diffusion_size_left, Size(0, 0), image_proc_scale, image_proc_scale);
  resize(current_image_right, diffusion_size_right, Size(0, 0), image_proc_scale, image_proc_scale);
  cvtColor(diffusion_size_left, diffusion_size_left, CV_BGR2GRAY);
  cvtColor(diffusion_size_right, diffusion_size_right, CV_BGR2GRAY);
  Mat l_diffused, r_diffused;
  boost::thread diffusion_L(anisotropic_diffusion, boost::cref(diffusion_size_left),
                            boost::ref(l_diffused), diffusion_time);
  boost::thread diffusion_R(anisotropic_diffusion, boost::cref(diffusion_size_right),
                            boost::ref(r_diffused), diffusion_time);
  diffusion_L.join();
  diffusion_R.join();

  // Extract Features
  vector< Point > features_l, features_r;
  int max_corners = 20;
  int block_size = 11;
  double quality_level = 0.05;
  double min_distance = 20.0;
  Mat l_diffused_draw = l_diffused.clone();
  Mat r_diffused_draw = r_diffused.clone();
  goodFeaturesToTrack(l_diffused, features_l, max_corners, quality_level, min_distance, Mat(), block_size);
  for(size_t i = 0; i < features_l.size(); i++){
    Point pt = features_l[i];
    circle(l_diffused_draw, pt, 2, Scalar(0), 2);
  }
  goodFeaturesToTrack(r_diffused, features_r, max_corners, quality_level, min_distance, Mat(), block_size);
  for(size_t i = 0; i < features_r.size(); i++){
    Point pt = features_r[i];
    circle(r_diffused_draw, pt, 2, Scalar(0), 2);
  }
  cout << "left features: " << features_l.size() << " right features: " << features_r.size() << endl;
  // imshow("left features", l_diffused_draw); waitKey(10);
  // imshow("right features", l_diffused_draw); waitKey(10);
  for (size_t i = 0; i < features_l.size(); i++){
    cout << "\x1b[32m" << i << " \x1b[0m" << features_l[i] << '\t' << features_r[i] << endl;
  }

  // GoodFeaturesToTrackDetector detector(max_corners, quality_level, min_distance, block_size);
  // Ptr<DescriptorExtractor> extractor = DescriptorExtractor::create("FREAK");
  // Ptr<DescriptorMatcher> matcher = DescriptorMatcher::create("BruteForce-Hamming");

  // std::vector<KeyPoint> keypoints_l, keypoints_r;

  // detector.detect( processing_size_image_right, keypoints_l );
  // detector.detect( processing_size_image_left, keypoints_r );

  // //-- Step 2: Calculate descriptors (feature vectors)
  // // FREAK extractor;

  // Mat descriptors_l, descriptors_r;

  // extractor->compute( processing_size_image_left, keypoints_l, descriptors_l );
  // extractor->compute( processing_size_image_right, keypoints_r, descriptors_r );

  // //-- Step 3: Matching descriptor vectors with a brute force matcher
  // // BFMatcher matcher(NORM_L2);
  // std::vector< DMatch > matches;
  // matcher->match( descriptors_l, descriptors_r, matches );

  // //-- Draw matches
  // Mat img_matches;
  // drawMatches( processing_size_image_right, keypoints_l, processing_size_image_left, keypoints_r, matches, img_matches );

  // //-- Show detected matches
  // imshow("Matches", img_matches );

  // waitKey(0);

  // TODO: Refine features for subpixel accuracy

  // Calculate stereo correspondence
  vector<int> correspondence_pair_idxs;
  // stereo_correspondence(l_diffused, r_diffused, features_l, features_r, correspondence_pair_idxs);

  // Dumb stereo matching
  cout << "Stereo matching..." << endl;
  {
    double curr_min_dist, xdiff, ydiff, dist;
    int curr_min_dist_idx;
    int y_diff_thresh = diffusion_size_left.rows * 0.02;
    cout << "y_diff_thresh: " << y_diff_thresh << endl;
    for (size_t i = 0; i < features_l.size(); i++){
      curr_min_dist_idx = -1;
      curr_min_dist = 1E6;
      cout << "\x1b[31m" << i << " \x1b[0mCurrent pt: "  << features_l[i] << endl;
      for(size_t j = 0; j < features_r.size(); j++){
        cout << "\t\x1b[31m" << j << " \x1b[0mCandidate pt: "  << features_r[j] << endl;
        ydiff = features_l[i].y - features_r[j].y;
        cout << "\t   ydiff: " << ydiff << endl;
        if(abs(ydiff) > y_diff_thresh) continue;
        xdiff = features_l[i].x - features_r[j].x;
        
        dist = sqrt(xdiff * xdiff + ydiff * ydiff);
        cout << "\t   dist: " << dist << endl;
        if(dist < curr_min_dist){
          curr_min_dist = dist;
          curr_min_dist_idx = j;
        }
      }
      correspondence_pair_idxs.push_back(curr_min_dist_idx);
      cout << "Match: " << curr_min_dist_idx << endl;
    }
  }

  // Print correspondences
  for (size_t i = 0; i < correspondence_pair_idxs.size(); i++){
    cout << i << " <--> " << correspondence_pair_idxs[i] << endl;
  }

  // Get camera projection matrices
  Matx34d left_cam_mat = left_cam_model.fullProjectionMatrix();
  Matx34d right_cam_mat = right_cam_model.fullProjectionMatrix();

  // Calculate 3D stereo reconstructions
  vector<Eigen::Vector3d> feature_pts_3d;
  Eigen::Vector3d pt_3D;
  double reset_scaling = 1 / image_proc_scale;
  cout << "feature reconstructions(3D):\n";
  for (size_t i = 0; i < correspondence_pair_idxs.size(); i++) {
    if(correspondence_pair_idxs[i] == -1) continue;
    Point2d pt_L = features_l[ i ];
    Point2d pt_R = features_r[ correspondence_pair_idxs[i] ];

    // Undo the effects of working with coordinates from scaled images
    pt_L = pt_L * reset_scaling;
    pt_R = pt_R * reset_scaling;

    // Print points in image coordinates
    cout << "L: " << pt_L << "R: " << pt_R << endl;

    Matx31d pt_L_hom(pt_L.x, pt_L.y, 1);
    Matx31d pt_R_hom(pt_R.x, pt_R.y, 1);
    Mat X_hom = sub::triangulate_Linear_LS(Mat(left_cam_mat),
                                           Mat(right_cam_mat),
                                           Mat(pt_L_hom), Mat(pt_R_hom));
    X_hom = X_hom / X_hom.at<double>(3, 0);
    pt_3D << 
      X_hom.at<double>(0, 0), X_hom.at<double>(1, 0), X_hom.at<double>(2, 0);
    cout << "[ " << pt_3D(0) << ", "  << pt_3D(1) << ", " << pt_3D(2) << "]" << endl;
    feature_pts_3d.push_back(pt_3D);
  }
  cout << "num 3D features: " << feature_pts_3d.size() << endl;

  // visualize reconstructions
  for(size_t i = 0; i < feature_pts_3d.size(); i++){
    Eigen::Vector3d pt = feature_pts_3d[i];
    Matx41d position_hom(pt(0), pt(1), pt(2), 1);
    Matx31d pt_L_2d_hom = left_cam_mat * position_hom;
    Point2d L_center2d(pt_L_2d_hom(0) / pt_L_2d_hom(2), pt_L_2d_hom(1) / pt_L_2d_hom(2));
    Scalar color(255, 0, 255);
    stringstream label;
    label << i;
    circle(current_image_left, L_center2d, 5, color, -1);
    putText(current_image_left, label.str(), L_center2d, FONT_HERSHEY_SIMPLEX,
            0.0015 * current_image_left.rows, Scalar(0, 0, 0), 2);
  }
  imshow("Detection", current_image_left); waitKey(1);

  // Pick a combination of four points that closely matches our model
  vector< vector<uint8_t> > four_pt_combo_idxs;
  combinations(feature_pts_3d.size(), 4, four_pt_combo_idxs);

  Eigen::Vector3d centroid;
  double model_height = 1.7;
  double model_width = 0.85;
  double dist_to_centroid_ideal = sqrt(model_height*model_height/4 + model_width*model_width/4);
  double sum_network_distances_ideal = 2*sqrt(model_height*model_height + model_width*model_width) + 2*model_height + 2*model_width;
  double xdiff, ydiff, zdiff;
  double dist_to_centroid_relative_error_thresh = 0.05;
  int curr_min_cost_idx = -1;
  double curr_min_cost = 1E9;
  cout << "dist_to_centroid_ideal: " << dist_to_centroid_ideal << endl;
  cout << "dist_to_centroid_relative_error_thresh: " << dist_to_centroid_relative_error_thresh << endl;
  cout << "sum_network_distances_ideal: " << sum_network_distances_ideal << endl;
  cout << "dist_to_centroid_relative_error_thresh: " << dist_to_centroid_relative_error_thresh << endl;
  vector<double> board_plane_coeffs;
  vector<Eigen::Vector3d> corrected_corners(4, Eigen::Vector3d());
  for(size_t i = 0; i < four_pt_combo_idxs.size(); i++){
    cout << i << " idxs: " << (int)four_pt_combo_idxs[i][0] << ", " << (int)four_pt_combo_idxs[i][1]
         << ", " << (int)four_pt_combo_idxs[i][2] << ", " << (int)four_pt_combo_idxs[i][3] << endl;
    
    // Calculate the avg distance to the centroid
    double xsum = 0;
    double ysum = 0;
    double zsum = 0;
    double dist_to_centroid_avg = 0;
    for(uint8_t pt_idx : four_pt_combo_idxs[i]){
      xsum += feature_pts_3d[pt_idx][0];
      ysum += feature_pts_3d[pt_idx][1];
      zsum += feature_pts_3d[pt_idx][2];
    }
    for(uint8_t pt_idx : four_pt_combo_idxs[i]){
      xdiff = feature_pts_3d[pt_idx][0] - xsum / 4.0;
      ydiff = feature_pts_3d[pt_idx][1] - ysum / 4.0;
      zdiff = feature_pts_3d[pt_idx][2] - zsum / 4.0;
      dist_to_centroid_avg += sqrt(xdiff * xdiff + ydiff * ydiff + zdiff * zdiff);
    }
    dist_to_centroid_avg /= 4.0;
    double dist_to_centroid_avg_error = fabs(dist_to_centroid_avg - dist_to_centroid_ideal) / dist_to_centroid_ideal;
    cout << "\tdist_to_centroid_avg: " << dist_to_centroid_avg << endl;
    cout << "\tdist_error: " << dist_to_centroid_avg_error << endl;
    // Disqualify if too far from model
    // if(dist_to_centroid_avg_error > dist_to_centroid_relative_error_thresh) continue;

    // Calc sum of all distances
    double model_matching_cost = 0;
    double sum_network_distances = 0;
    vector<double> network_distances, model_match_costs;
    vector< vector<uint8_t> > four_choose_2;
    combinations(4, 2, four_choose_2);
    for(vector<uint8_t> pair_idx : four_choose_2){
      double dist = (feature_pts_3d[four_pt_combo_idxs[i][pair_idx[0]]] - feature_pts_3d[four_pt_combo_idxs[i][pair_idx[1]]]).norm(); 
      network_distances.push_back(dist);
      sum_network_distances += dist;
      cout << "\t\t\x1b[32mdist:" << dist << endl << "\x1b[0m";
    }
    cout << "\t\t\x1b[32msum_network_distances:" << sum_network_distances << endl << "\x1b[0m";
    // Disqualify if too far from model
    // if(sum_network_distances_relative_error > sum_network_distances_relative_error_thresh) continue;
    
    // Calculate a cost function for departure from model
    sort(network_distances.begin(), network_distances.end());
    // Compare graph edge lengths with expectations
    model_match_costs.push_back(pow(network_distances[0] - model_width, 2.0));
    model_match_costs.push_back(pow(network_distances[1] - model_width, 2.0));
    model_match_costs.push_back(pow(network_distances[2] - model_height, 2.0));
    model_match_costs.push_back(pow(network_distances[3] - model_height, 2.0));
    model_match_costs.push_back(pow(network_distances[4] - sqrt(model_height*model_height + model_width*model_width), 2.0));
    model_match_costs.push_back(pow(network_distances[5] - sqrt(model_height*model_height + model_width*model_width), 2.0));
    for(double cost : model_match_costs){
      cout << "\tcost: " << cost << endl;
      model_matching_cost += cost;
    }
    // Add cost for departure from expected right angle corners
    double orthogonality_measure = 0;
    double orthogonality_weight = 1.0;
    for(size_t j = 0; j < four_pt_combo_idxs[i].size(); j++){
      // eliminate most distant point
      int max_dist_idx = j;
      int max_dist = -1;
      for(size_t k = 0; k < four_pt_combo_idxs[i].size(); k++){
        if(j == k) continue;
        double dist = (feature_pts_3d[four_pt_combo_idxs[i][j]] - feature_pts_3d[four_pt_combo_idxs[i][k]]).norm(); 
        if(dist < max_dist){
          max_dist_idx = k;
          max_dist = dist;
        }      
      }
      // vectors formed with two other points should have zero dot product
      Eigen::Vector3d v1, v2;
      bool first = true;
      for(size_t k = 0; k < four_pt_combo_idxs[i].size(); k++){
        if(k == j || k == (size_t)max_dist_idx) continue;
        if(first){
          v1 = feature_pts_3d[four_pt_combo_idxs[i][k]] - feature_pts_3d[four_pt_combo_idxs[i][j]];
          first = false;
        }
        else v2 = feature_pts_3d[four_pt_combo_idxs[i][k]] - feature_pts_3d[four_pt_combo_idxs[i][j]];
      }
      //  orthogonality_measure will be zero if perfectly orthogonal
      orthogonality_measure += fabs(v1.dot(v2) / (v1.norm() * v2.norm()));
    }
    orthogonality_measure /= 4.0;
    // increase cost if not close to right angle corners
    model_matching_cost *= (1.0 + orthogonality_weight * orthogonality_measure); 
    cout << "orthogonality_measure: " << orthogonality_measure << endl;
  
    cout << "\t\t\x1b[32mmodel_matching_cost:" << model_matching_cost << endl << "\x1b[0m";
    if(model_matching_cost < curr_min_cost){
      curr_min_cost = model_matching_cost;
      curr_min_cost_idx = i;
    }
  }
  cout << "min_cost_idx: " << curr_min_cost_idx << " min_cost: " << curr_min_cost << endl;
  if(curr_min_cost < 0.05){
    cout << "Model found at index: " << curr_min_cost_idx << endl;
  }
  else{
    cout << "Model not found" << endl;
    return;
  }
  

  // Calculate best fit plane
  Eigen::Matrix<double, 4, 3> A;
  Eigen::Matrix<double, 4, 1> b_vec;
  A <<  feature_pts_3d[four_pt_combo_idxs[curr_min_cost_idx][0]][0], feature_pts_3d[four_pt_combo_idxs[curr_min_cost_idx][0]][1], feature_pts_3d[four_pt_combo_idxs[curr_min_cost_idx][0]][2],
        feature_pts_3d[four_pt_combo_idxs[curr_min_cost_idx][1]][0], feature_pts_3d[four_pt_combo_idxs[curr_min_cost_idx][1]][1], feature_pts_3d[four_pt_combo_idxs[curr_min_cost_idx][1]][2],
        feature_pts_3d[four_pt_combo_idxs[curr_min_cost_idx][2]][0], feature_pts_3d[four_pt_combo_idxs[curr_min_cost_idx][2]][1], feature_pts_3d[four_pt_combo_idxs[curr_min_cost_idx][2]][2],
        feature_pts_3d[four_pt_combo_idxs[curr_min_cost_idx][3]][0], feature_pts_3d[four_pt_combo_idxs[curr_min_cost_idx][3]][1], feature_pts_3d[four_pt_combo_idxs[curr_min_cost_idx][3]][2];
  b_vec << 1, 1, 1, 1;
  Eigen::Matrix<double, 3, 1> x = A.colPivHouseholderQr().solve(b_vec);
  double a, b, c, d;
  a = 1;
  b = x[1] / x[0];
  c = x[2] / x[0];
  d = -1 / x[0];
  cout << "best fit plane: z = " << -a/c << "x + " << -b/c << "y + " << -d/c << endl;
  
  // Project points to best fit plane
  vector<Eigen::Vector3d> proj_pts;
  Eigen::Vector3d plane_unit_normal;
  plane_unit_normal << a, b, c;
  plane_unit_normal = plane_unit_normal / plane_unit_normal.norm();
  Eigen::Vector3d pt_on_plane;
  pt_on_plane << 0, 0, -d/c;
  for(uint8_t pt_idx : four_pt_combo_idxs[curr_min_cost_idx]){
    Eigen::Vector3d pt = feature_pts_3d[pt_idx];
    Eigen::Vector3d plane_to_pt_vec = pt - pt_on_plane;
    Eigen::Vector3d plane_to_pt_proj_normal = plane_to_pt_vec.dot(plane_unit_normal) * plane_unit_normal;
    Eigen::Vector3d corr_pt = pt - plane_to_pt_proj_normal;
    cout << (int)pt_idx << ":\noriginal pt: [" << pt[0] << ", " << pt[1] << ", " << pt[2] << "] \ncorrected: ["
         << corr_pt[0] << ", " << corr_pt[1] << ", " << corr_pt[2] << "]\n\tdist: "
         << (pt - corr_pt).norm() << endl;
    proj_pts.push_back(corr_pt);
  }

  Eigen::Vector3d long_vec, short_vec;
  for(int k = 1; k < 4; k++){
    double dist = (proj_pts[0] - proj_pts[k]).norm();
    if(fabs(dist - model_height) < model_height * 0.075) long_vec = proj_pts[k] - proj_pts[0];
    if(fabs(dist - model_width) < model_width * 0.075) short_vec = proj_pts[k] - proj_pts[0];
  }
  cout << "\tarea = " << short_vec.cross(long_vec).norm() << endl;
  // if(fabs(short_vec.cross(long_vec).norm() - 0.7564) > 0.05 * 0.7564) continue;
  corrected_corners[0] = proj_pts[0];
  corrected_corners[1] = proj_pts[0] + long_vec;
  corrected_corners[2] = proj_pts[0] + long_vec + short_vec;
  corrected_corners[3] = proj_pts[0] + short_vec;




  // Generate plane defined by three points included in "feature_pts_3d" that 
  // best includes the most points within a threshold
  // vector<double> board_plane_coeffs(4, 0);
  // double distance_threshold = 0.25;
  // best_plane_from_combination(feature_pts_3d, distance_threshold, board_plane_coeffs);

  // // Threshold points that fall within a distance of previously determined plane
  // double dist_threshold = 0.25;
  // vector<double> pt_plane_distances;
  // vector<Eigen::Vector3d> threshed_features;
  // cout << "threshing distances:\n";
  // for(Eigen::Vector3d pt : feature_pts_3d){
  //   double distance = point_to_plane_distance(board_plane_coeffs[0], board_plane_coeffs[1],
  //                                             board_plane_coeffs[2], board_plane_coeffs[3], pt);
  //   cout << distance << " ";
  //   if(distance < dist_threshold) threshed_features.push_back(pt);
  // }

  // TODO: find best fit plane to threshed points

  // // Determine indices for each possible pair from the threshed points
  // vector< vector<uint8_t> > pt_pair_idxs;
  // combinations(threshed_features.size(), 2, pt_pair_idxs);

  // // Calculate distances between all previous pairs
  // vector<double> pt_pair_dists;
  // cout << "\npair distances:\n";
  // for(vector<uint8_t> pair_idx : pt_pair_idxs){
  //   double distance = (threshed_features[pair_idx[0]] - threshed_features[pair_idx[1]]).norm();
  //   pt_pair_dists.push_back(distance);
  //   cout << distance << " ";
  // }

  for(int idx : four_pt_combo_idxs[curr_min_cost_idx]){
    Eigen::Vector3d pt = feature_pts_3d[idx];
    Matx41d position_hom(pt(0), pt(1), pt(2), 1);
    Matx31d pt_L_2d_hom = left_cam_mat * position_hom;
    Point2d L_center2d(pt_L_2d_hom(0) / pt_L_2d_hom(2), pt_L_2d_hom(1) / pt_L_2d_hom(2));
    Scalar color(0, 0, 255);
    circle(current_image_left, L_center2d, 5, color, -1);
  }

  // for(Eigen::Vector3d pt : threshed_features){
  //   Matx41d position_hom(pt(0), pt(1), pt(2), 1);
  //   Matx31d pt_L_2d_hom = left_cam_mat * position_hom;
  //   Point2d L_center2d(pt_L_2d_hom(0) / pt_L_2d_hom(2), pt_L_2d_hom(1) / pt_L_2d_hom(2));
  //   Scalar color(255, 0, 255);
  //   circle(current_image_left, L_center2d, 5, color, -1);
  // }
  // imshow("reprojected stereo features", current_image_left);

  // // Calculate pair of points with distance closest to height
  // vector<double> pair_model_edge_diffs(pt_pair_idxs.size(), 0);
  // for (size_t i = 0; i < pt_pair_idxs.size(); i++){
  //   Eigen::Vector3d pt1 = threshed_features[pt_pair_idxs[i][0]];
  //   Eigen::Vector3d pt2 = threshed_features[pt_pair_idxs[i][1]];
  //   double abs_diff_from_model = 1.24 - (pt1 - pt2).norm();
  //   abs_diff_from_model = sqrt(abs_diff_from_model * abs_diff_from_model);
  //   pair_model_edge_diffs[i] = abs_diff_from_model;
  // }

  // // Snap pair of features to left or right edge of model
  // vector<uint8_t> pair_idxs;
  // int edge_match_idx = distance(pair_model_edge_diffs.begin(),
  //                               min_element(pair_model_edge_diffs.begin(),
  //                                           pair_model_edge_diffs.end()));
  // pair_idxs = pt_pair_idxs[edge_match_idx];
  // if(pair_model_edge_diffs[edge_match_idx] > 1.24 * 0.1) return;

  // Eigen::Vector3d pt_TL, pt_BL;
  // if(threshed_features[pair_idxs[0]][2] > threshed_features[pair_idxs[1]][2]){
  //   pt_TL = threshed_features[pair_idxs[0]];
  //   pt_BL = threshed_features[pair_idxs[1]];
  // }
  // else{
  //   pt_TL = threshed_features[pair_idxs[1]];
  //   pt_BL = threshed_features[pair_idxs[0]];
  // }

  // Eigen::Vector3d plane_normal;
  // plane_normal << board_plane_coeffs[0], board_plane_coeffs[1], board_plane_coeffs[2];
  // Eigen::Vector3d up_left_vec = pt_TL - pt_BL;
  // Eigen::Vector3d top_bottom_edge_vec;
  // top_bottom_edge_vec = up_left_vec.cross(plane_normal);
  // top_bottom_edge_vec = top_bottom_edge_vec / top_bottom_edge_vec.norm();
  // top_bottom_edge_vec = top_bottom_edge_vec * 0.61;
  // Eigen::Vector3d pt_TR = pt_TL + top_bottom_edge_vec;
  // Eigen::Vector3d pt_BR = pt_BL + top_bottom_edge_vec;

  Eigen::Vector3d pt_TL, pt_BL, pt_BR, pt_TR;
  pt_TL = corrected_corners[0];
  pt_BL = corrected_corners[1];
  pt_BR = corrected_corners[2];
  pt_TR = corrected_corners[3];

  // double area = up_left_vec.cross(top_bottom_edge_vec).norm();
  // cout << "\nAREA: " << area << " m^2" << endl;

  // if((area > 1.1 * 1.24 * 0.61) || (area < 0.9 * 1.24 * 0.61)) return;

  Matx41d position_hom(pt_TL(0), pt_TL(1), pt_TL(2), 1);
  Matx31d pt_L_2d_hom = left_cam_mat * position_hom;
  Point2d pt_TL_2d(pt_L_2d_hom(0) / pt_L_2d_hom(2), pt_L_2d_hom(1) / pt_L_2d_hom(2));

  position_hom = Matx41d(pt_TR(0), pt_TR(1), pt_TR(2), 1);
  pt_L_2d_hom = left_cam_mat * position_hom;
  Point2d pt_TR_2d(pt_L_2d_hom(0) / pt_L_2d_hom(2), pt_L_2d_hom(1) / pt_L_2d_hom(2));

  position_hom = Matx41d(pt_BR(0), pt_BR(1), pt_BR(2), 1);
  pt_L_2d_hom = left_cam_mat * position_hom;
  Point2d pt_BR_2d(pt_L_2d_hom(0) / pt_L_2d_hom(2), pt_L_2d_hom(1) / pt_L_2d_hom(2));

  position_hom = Matx41d(pt_BL(0), pt_BL(1), pt_BL(2), 1);
  pt_L_2d_hom = left_cam_mat * position_hom;
  Point2d pt_BL_2d(pt_L_2d_hom(0) / pt_L_2d_hom(2), pt_L_2d_hom(1) / pt_L_2d_hom(2));
  Scalar color(255, 0, 255);

  line(current_image_left, pt_TL_2d, pt_TR_2d, color, 2);
  line(current_image_left, pt_TR_2d, pt_BR_2d, color, 2);
  line(current_image_left, pt_BR_2d, pt_BL_2d, color, 2);
  line(current_image_left, pt_BL_2d, pt_TL_2d, color, 2);

  // imshow("Detection", current_image_left); waitKey(1);






  cout << "finished processing!" << endl;
  // imshow("left_denoised", processing_size_image_left); waitKey(1);
  return;

  if (generate_dbg_img) {
    try {
      processing_size_image_left.copyTo(debug_image(lower_left));
      processing_size_image_right.copyTo(debug_image(lower_right));
    } catch (exception &e) {
      stringstream error_msg;
      error_msg << "Exception drawing processing frame on debug image quadrant."
                << e.what() << endl
                << "processing_size_image_left size: "
                << processing_size_image_left.size()
                << " upper_left size: " << upper_left.size() << endl
                << "processing_size_image_right size: "
                << processing_size_image_right.size()
                << " upper_right size: " << upper_right.size() << endl
                << "processing img depth:"
                << processing_size_image_left.channels()
                << " dbg_img depth:" << debug_image.channels() << endl
                << "processing img type:" << processing_size_image_left.type()
                << " dbg_img type:" << debug_image.type();
      ROS_WARN(error_msg.str().c_str());
    }
  }

  // Segment Board and find image coordinates of board corners
  Mat left_segment_dbg_img, right_segment_dbg_img;
  segment_board(processing_size_image_left, segmented_board_left,
                left_segment_dbg_img, false);
  segment_board(processing_size_image_right, segmented_board_right,
                right_segment_dbg_img, true);
#ifdef SEGMENTATION_DEBUG
  imshow("segmented right", segmented_board_right);
  waitKey(1);
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
      Mat ll_dbg = debug_image(lower_left);
      Mat lr_dbg = debug_image(lower_right);
      Point L_img_corner = left_corners[i] * image_proc_scale;
      Point R_img_corner = right_corners[i] * image_proc_scale;
      Scalar color(0, 0, 0);
      switch (i) {
      case 0:
        circle(ll_dbg, L_img_corner, ll_dbg.rows / 50.0, color, -1);
        circle(lr_dbg, R_img_corner, ll_dbg.rows / 50.0, color, -1);
        break;
      case 1:
        color = Scalar(0, 0, 255);
        circle(ll_dbg, L_img_corner, ll_dbg.rows / 50.0, color, -1);
        circle(lr_dbg, R_img_corner, ll_dbg.rows / 50.0, color, -1);
        break;
      case 2:
        color = Scalar(0, 255, 0);
        circle(ll_dbg, L_img_corner, ll_dbg.rows / 50.0, color, -1);
        circle(lr_dbg, R_img_corner, ll_dbg.rows / 50.0, color, -1);
        break;
      case 3:
        color = Scalar(255, 0, 0);
        circle(ll_dbg, L_img_corner, ll_dbg.rows / 50.0, color, -1);
        circle(lr_dbg, R_img_corner, ll_dbg.rows / 50.0, color, -1);
        break;
      }
    }
  }

  // Reconstruct 3d corners from corresponding image points
  vector<Eigen::Vector3d> corners_3d;
  Eigen::Vector3d X;
  for (int i = 0; i < 4; i++) {
    Point2d pt_L_ = left_corners[i];
    Point2d pt_R_ = right_corners[i];
    Matx31d pt_L(pt_L_.x, pt_L_.y, 1);
    Matx31d pt_R(pt_R_.x, pt_R_.y, 1);
    Mat X_hom = sub::triangulate_Linear_LS(Mat(left_cam_mat),
                                               Mat(right_cam_mat),
                                               Mat(pt_L), Mat(pt_R));
    X_hom = X_hom / X_hom.at<double>(3, 0);
    X << 
      X_hom.at<double>(0, 0), X_hom.at<double>(1, 0), X_hom.at<double>(2, 0);
    corners_3d.push_back(X);
  }

  // Calculate 3d board position (center of board)
  Eigen::Vector3d position(0, 0, 0);
  dbg_str << "3D Corners:" << endl;
  for (int i = 0; i < 4; i++){
    dbg_str << corners_3d[i].transpose() << endl;
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
  vector<Eigen::Vector3d> targets;
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
  string tf_frame = left_cam_model.tfFrame();
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
  catch(exception &e){
    ROS_INFO_THROTTLE(5, e.what());
  }

  // Project 3d centroid and targets into both images and visualize
  if (generate_dbg_img) {
    Matx41d position_hom(position(0), position(1), position(2), 1);
    Matx41d target1_hom(targets[0](0), targets[0](1), targets[0](2), 1);
    Matx41d target2_hom(targets[1](0), targets[1](1), targets[1](2), 1);
    Matx31d L_center2d_hom = left_cam_mat * position_hom;
    Matx31d R_center2d_hom = right_cam_mat * position_hom;
    Matx31d L_target1_hom = left_cam_mat * target1_hom;
    Matx31d R_target1_hom = right_cam_mat * target1_hom;
    Matx31d L_target2_hom = left_cam_mat * target2_hom;
    Matx31d R_target2_hom = right_cam_mat * target2_hom;
    Point2d L_center2d(
        L_center2d_hom(0) / L_center2d_hom(2),
        L_center2d_hom(1) / L_center2d_hom(2));
    Point2d R_center2d(
        R_center2d_hom(0) / R_center2d_hom(2),
        R_center2d_hom(1) / R_center2d_hom(2));
    Point2d L_target1(
        L_target1_hom(0) / L_target1_hom(2),
        L_target1_hom(1) / L_target1_hom(2));
    Point2d R_target1(
        R_target1_hom(0) / R_target1_hom(2),
        R_target1_hom(1) / R_target1_hom(2));
    Point2d L_target2(
        L_target2_hom(0) / L_target2_hom(2),
        L_target2_hom(1) / L_target2_hom(2));
    Point2d R_target2(
        R_target2_hom(0) / R_target2_hom(2),
        R_target2_hom(1) / R_target2_hom(2));
    Mat ll_dbg = debug_image(lower_left);
    Mat lr_dbg = debug_image(lower_right);
    Scalar color(255, 0, 255);
    circle(ll_dbg, L_center2d * image_proc_scale, 5, color, -1);
    color = Scalar(255, 0, 255);
    circle(lr_dbg, R_center2d * image_proc_scale, 5, color, -1);
    color = Scalar(255, 255, 0);
    circle(ll_dbg, L_target1 * image_proc_scale, 5, color, -1);
    circle(lr_dbg, R_target1 * image_proc_scale, 5, color, -1);
    circle(ll_dbg, L_target2 * image_proc_scale, 5, color, -1);
    circle(lr_dbg, R_target2 * image_proc_scale, 5, color, -1);
    dbg_str << "centroid_3d: " << position.transpose() << endl;
    dbg_str << "centroid_dbg_img_coords_left: "
            << L_center2d *image_proc_scale << endl;
    dbg_str << "centroid_dbg_img_coords_right: "
            << R_center2d *image_proc_scale << endl;

    stringstream left_text, right_text;
    int height = debug_image(lower_left).rows;
    Point header_text_pt(height / 20.0, height / 10.0);
    left_text << "Left  " << left_most_recent.image_msg_ptr->header.stamp;
    right_text << "Right " << right_most_recent.image_msg_ptr->header.stamp;
    int font = FONT_HERSHEY_SIMPLEX;
    double font_scale = 0.0015 * height;
    putText(ll_dbg, left_text.str(), header_text_pt, font,
                font_scale, color);
    putText(lr_dbg, right_text.str(), header_text_pt, font,
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

void Sub8TorpedoBoardDetector::segment_board(const Mat &src, Mat &dest,
                                             Mat &dbg_img,
                                             bool draw_dbg_img) {

  // Preprocessing
  Mat hsv_image, hue_segment_dbg_img, sat_segment_dbg_img;
  vector<Mat> hsv_channels;
  cvtColor(src, hsv_image, CV_BGR2HSV);
  split(hsv_image, hsv_channels);

  // Histogram parameters
  int hist_size = 256; // bin size
  float range[] = {0, 255};
  const float *ranges[] = {range};

  // Segment out torpedo board
  Mat threshed_hue, threshed_sat, segmented_board;
  int target_yellow = 20;
  int target_saturation = 180;
  sub::statistical_image_segmentation(
      hsv_channels[0], threshed_hue, hue_segment_dbg_img, hist_size, ranges,
      target_yellow, "Hue", draw_dbg_img, 6, 3.0, 3.0);
  sub::statistical_image_segmentation(
      hsv_channels[1], threshed_sat, sat_segment_dbg_img, hist_size, ranges,
      target_saturation, "Saturation", draw_dbg_img, 6, 0.1, 0.1);
#ifdef SEGMENTATION_DEBUG
  imshow("segment board input", src);
  imshow("hue segment", threshed_hue);
  imshow("sat segment", threshed_sat);
  waitKey(1);
#endif
  segmented_board = threshed_hue / 2.0 + threshed_sat / 2.0;
  medianBlur(segmented_board, segmented_board, 5);
  threshold(segmented_board, segmented_board, 255 * 0.9, 255,
                THRESH_BINARY);
  dest = segmented_board;

  if (generate_dbg_img && draw_dbg_img) {
    try {
      hue_segment_dbg_img.copyTo(debug_image(upper_left));
      sat_segment_dbg_img.copyTo(debug_image(upper_right));
    } catch (exception &e) {
      stringstream error_msg;
      error_msg << "Exception thrown drawing segmentation sbg image on "
                   "debug_image quadrant" << endl
                << "hue_segment_dbg_img size: " << hue_segment_dbg_img.size()
                << " upper_left size: " << upper_left.size() << endl
                << "sat_segment_dbg_img size: " << sat_segment_dbg_img.size()
                << " upper_right size: " << upper_right.size() << endl
                << "processing img depth: " << hue_segment_dbg_img.channels()
                << " dbg_img depth: " << debug_image.channels() << endl
                << "processing img type: " << hue_segment_dbg_img.type()
                << " dbg_img type: " << debug_image.type() << endl
                << e.what();
      ROS_WARN(error_msg.str().c_str());
    }
  }
}

bool Sub8TorpedoBoardDetector::find_board_corners(
    const Mat &segmented_board, vector<Point> &corners, bool draw_dbg_left) {
  bool corners_success = false;
  corners.clear();
  Mat convex_hull_working_img = segmented_board.clone();
  vector< vector<Point> > contours, connected_contours;
  vector<Point> convex_hull, corner_points;

  /// Find contours
  findContours(convex_hull_working_img, contours, CV_RETR_EXTERNAL,
                   CV_CHAIN_APPROX_SIMPLE);

  // Put longest 2 contours at beginning of "contours" vector
  if (contours.size() > 2) {
    partial_sort(contours.begin(), contours.begin() + 2, contours.end(),
                      sub::larger_contour);
  }
  if (contours.size() < 2) { // prevent out of range access of contours
    corners_success = false;
    return corners_success;
  }

  // Draw preliminary edge of torpedo_board on debug_image
  if (generate_dbg_img) {
    if (draw_dbg_left) { // Draw on lower left quadrant
      drawContours(debug_image(lower_left), contours, 0,
                       Scalar(255, 255, 255), 2);
      drawContours(debug_image(lower_left), contours, 1,
                       Scalar(255, 255, 255), 2);
    } else { // Draw on lower right quadrant
      drawContours(debug_image(lower_right), contours, 0,
                       Scalar(255, 255, 255), 2);
      drawContours(debug_image(lower_right), contours, 1,
                       Scalar(255, 255, 255), 2);
    }
  }

  // Connect yellow pannels
  Point pt1 = sub::contour_centroid(contours[0]);
  Point pt2 = sub::contour_centroid(contours[1]);
  convex_hull_working_img =
      Mat::zeros(convex_hull_working_img.size(), CV_8UC1);
  drawContours(convex_hull_working_img, contours, 0, Scalar(255));
  drawContours(convex_hull_working_img, contours, 1, Scalar(255));
  line(convex_hull_working_img, pt1, pt2, Scalar(255));
  findContours(convex_hull_working_img, connected_contours,
                   CV_RETR_EXTERNAL, CV_CHAIN_APPROX_SIMPLE);

  // Put longest contour at beginning of "connected_contours" vector
  if (contours.size() > 1) {
    partial_sort(connected_contours.begin(),
                      connected_contours.begin() + 1, connected_contours.end(),
                      sub::larger_contour);
  }

  // Find convex hull of connected panels
  convexHull(connected_contours[0], convex_hull);
  size_t poly_pts = convex_hull.size();
  const int max_iterations = 50;
  int total_iterations = 0;
  double epsilon = 1;
  double epsilon_step = 0.5;
  for (int i = 0; i < max_iterations; i++) {
    approxPolyDP(convex_hull, convex_hull, epsilon, true);
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
    stringstream ros_log;
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
  vector<float> convex_hull_angles;
  for(int i = 0; i < 4; i++){
    float x = convex_hull[i].x - center_x;
    float y = -convex_hull[i].y + center_y; // undoes effect of downward y in CV
    convex_hull_angles.push_back(atan2(y, x));
  }
  // Sorting by descending corresponding angle ensures this order
  vector<bool> pushed(4, false);
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
  // for(int i = 0; i < 4; i++) cout << "\x1b[33m" << corners[i] << endl << "\x1b[0m";
  return corners_success;
}

void Sub8TorpedoBoardDetector::stereo_correspondence(const Mat &gray_L, const Mat &gray_R,
                           const vector< Point > &features_L,
                           const vector< Point > &features_R,
                           vector< vector<int> > &corresponding_feats_idxs){
/*
  This function takes in two vectors of features in the form of cv::Point along with
  the grayscale image they were extracted from. It will attempt to features in the left
  image with the corresponding point in the right iage, if such point can be found. The
  output will be a vector that holds 2 element vectors with indices for point 
  correspondences. 

  For example:
    features_L[ corresponding_feats_idxs[i][0] ] and features_R[ corresponding_feats_idxs[i][1] ]
    are the ith pair of corresponding points found by this stereo matcher.
*/

  // Corresponding points should have approximately the same y-coordinate and somewhat similar x
  int y_diff_thresh = 0.025 * gray_L.rows;
  int x_diff_thresh = 0.050 * gray_L.rows;

  // Size of comparison block should be odd
  int block_size = gray_L.rows / 30;
  block_size = block_size + (block_size % 2) - 1;

  // Exclude features whose blocks cross the edge of the image
  vector<size_t> feat_L_idxs, feat_R_idxs;
  for(size_t i = 0; i < features_L.size(); i++){
    Point L = features_L[i];
    if(L.x - block_size / 2 < 0) continue;
    if(L.x + block_size / 2 > gray_L.cols - 1) continue;
    if(L.y - block_size / 2 < 0) continue;
    if(L.y + block_size / 2 > gray_L.rows - 1) continue;
    feat_L_idxs.push_back(i);
  }
  for(size_t i = 0; i < features_R.size(); i++){
    Point R = features_R[i];
    if(R.x - block_size / 2 < 0) continue;
    if(R.x + block_size / 2 > gray_R.cols - 1) continue;
    if(R.y - block_size / 2 < 0) continue;
    if(R.y + block_size / 2 > gray_R.rows - 1) continue;
    feat_R_idxs.push_back(i);
  }

  // Generate comparison blocks around feature pixels
  vector<Mat> blocks_L, blocks_R;
  for(size_t idx : feat_L_idxs){
    Point L = features_L[idx];
    Rect roi = Rect(L.x - block_size / 2, L.y - block_size / 2, block_size, block_size);
    blocks_L.push_back(gray_L(roi));
  }
  for(size_t idx : feat_R_idxs){
    Point R = features_R[idx];
    Rect roi = Rect(R.x - block_size / 2, R.y - block_size / 2, block_size, block_size);
    blocks_R.push_back(gray_R(roi));
  }

  // Points from smaller fature list will be matched to points from the
  // larger feature list (lists could be of equal size)
  vector<size_t> *small_idxs_ptr, *large_idxs_ptr;
  vector<Mat> *small_blocks_ptr, *large_blocks_ptr;
  bool left_list_smaller_or_equal;
  if(feat_L_idxs.size() <= feat_R_idxs.size()){
    small_idxs_ptr = &feat_L_idxs;
    large_idxs_ptr = &feat_R_idxs;
    small_blocks_ptr = &blocks_L;
    large_blocks_ptr = &blocks_R;
    left_list_smaller_or_equal = true;
  }
  else{
    small_idxs_ptr = &feat_R_idxs;
    large_idxs_ptr = &feat_L_idxs;
    small_blocks_ptr = &blocks_R;
    large_blocks_ptr = &blocks_L;
    left_list_smaller_or_equal = false;
  }

  // Compute dissimilarity measure for pairs of feature blocks
  int dissimilarity_thresh = 1000;
  int init = block_size * block_size * 25;
  vector< vector<int> > dissimilarity_measurements(small_idxs_ptr->size(), 
                                                   vector<int>(large_idxs_ptr->size(), init));
  for(size_t i = 0; i < small_idxs_ptr->size(); i++){
    for(size_t j = 0; j < large_idxs_ptr->size(); j++){
      Point small_list_pt = (left_list_smaller_or_equal? features_L[ (*small_idxs_ptr)[i] ] :
        features_R[ (*small_idxs_ptr)[i] ]);
      Point large_list_pt = (left_list_smaller_or_equal? features_R[ (*large_idxs_ptr)[i] ] :
        features_L[ (*large_idxs_ptr)[i] ]);
      if(std::abs(small_list_pt.x - large_list_pt.x) > x_diff_thresh) continue;
      if(std::abs(small_list_pt.y - large_list_pt.y) > y_diff_thresh) continue;
      Mat block_abs_diff;
      absdiff((*small_blocks_ptr)[i], (*large_blocks_ptr)[j], block_abs_diff);
      int dissimilarity = cv::sum(block_abs_diff)[0];
      if(dissimilarity > dissimilarity_thresh) continue;
      dissimilarity_measurements[i][j] = dissimilarity;
    }
  }

  // Determine match with minimum dissimilarity
  for(size_t i = 0; i < small_idxs_ptr->size(); i++){
    int min_dissimilarity_match_idx = 
      distance(dissimilarity_measurements[i].begin(),
               min_element(dissimilarity_measurements[i].begin(),
                           dissimilarity_measurements[i].end()));
    if(dissimilarity_measurements[i][min_dissimilarity_match_idx] == -1) continue;

    int correspondence_idx_L =
      (left_list_smaller_or_equal? (*small_idxs_ptr)[i] : (*large_idxs_ptr)[i]);
    int correspondence_idx_R =
      (left_list_smaller_or_equal? (*large_idxs_ptr)[i] : (*small_idxs_ptr)[i]);

    vector<int> correspondence_idxs;
    correspondence_idxs.push_back(correspondence_idx_L);
    correspondence_idxs.push_back(correspondence_idx_R);
    corresponding_feats_idxs.push_back(correspondence_idxs);
  }
}



///////////////////////////////////////////////////////////////////////////////////////////////////
// Class: TorpedoBoardReprojectionCost ////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////////////////

TorpedoBoardReprojectionCost::TorpedoBoardReprojectionCost(
    Matx34d &proj_mat_L, Matx34d &proj_mat_R,
    vector<Point> &image_corners_L,
    vector<Point> &image_corners_R)
    : proj_L(proj_mat_L), proj_R(proj_mat_R), img_corners_L(image_corners_L),
      img_corners_R(image_corners_R) {
  if (image_corners_L.size() != 4 || image_corners_R.size() != 4) {
    throw invalid_argument("Corner vectors should contain 4 points.");
  }
}

TorpedoBoardReprojectionCost::~TorpedoBoardReprojectionCost() {
  cout << "TorpedoBoardReprojectionCost being destructed." << endl;
}

vector<Point> TorpedoBoardReprojectionCost::getProjectedCorners(
    double center_x, double center_y, double center_z, double yaw,
    Matx34d &proj_matrix) {

  // Calculate 3d Corners
  vector<Matx41d> corners3d;
  double x, y, z;
  x = center_x - 0.5 * width_m * cos(yaw);
  y = center_y - 0.5 * height_m;
  z = center_z - 0.5 * width_m * sin(yaw);
  corners3d.push_back(Matx41d(x, y, z, 1.0));
  x = center_x + 0.5 * width_m * cos(yaw);
  z = center_z + 0.5 * width_m * sin(yaw);
  corners3d.push_back(Matx41d(x, y, z, 1.0));
  y = center_y + 0.5 * height_m;
  corners3d.push_back(Matx41d(x, y, z, 1.0));
  x = center_x - 0.5 * width_m * cos(yaw);
  z = center_z - 0.5 * width_m * sin(yaw);
  corners3d.push_back(Matx41d(x, y, z, 1.0));
  // untested !

  // Project Corners
  vector<Point> proj_corners;
  Matx31d proj_hom;
  for (size_t i = 0; i < 4; i++) {
    proj_hom = proj_matrix * corners3d[i];
    proj_corners.push_back(Point(proj_hom(0, 0) / proj_hom(2, 0),
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
  vector<Point2d> model_corners_L =
      getProjectedCorners(x, y, z, yaw, proj_L);

  // Calculate reprojection error from right image
  vector<Point2d> model_corners_R =
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

// vector<Point2d> project_model(Eigen::Matrix<double, 3, 4> cam_matx, Eigen::Vector3d position, Eigen::Quaterniond orientation){
//   // all units in meters
//   // x and y same as image basis vectors, z follows right hand rule
//   // origin is at center of board

//   typedef Eigen::Vector3d Point;
//   double height = 1.24;
//   double width = 0.61; 

//   corners[0] = Point(-width / 2, -height / 2, 0);  // top left
//   corners[1] = Point(width / 2, -height / 2, 0);   // top right
//   corners[2] = Point(width / 2, height / 2, 0);    // bottom right
//   corners[3] = Point(-width / 2, height / 2, 0);   // bottom left


// }

///////////////////////////////////////////////////////////////////////////////////////////////////
// Helper Functions ///////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////////////////

void combinations(uint8_t n, uint8_t k, vector< vector<uint8_t> > &idx_array){

  idx_array = vector< vector<uint8_t> >();
  
  vector<uint8_t> first_comb;
  
  // set first combination indices
  for(uint8_t i = 0; i < k; i++){
    first_comb.push_back(i);
  }

  uint8_t level = 0;

  _increase_elements_after_level(first_comb, idx_array, n, k, level);
}

void _increase_elements_after_level(vector<uint8_t> comb, vector< vector<uint8_t> > &comb_array,
                                   uint8_t n, uint8_t k, uint8_t level){

    vector<uint8_t> parent = comb;
    vector< vector<uint8_t> > children;

    while(true){

        for(uint8_t idx = level; idx < k; idx++){ comb[idx] = comb[idx] + 1; }
        if(comb[level] > n - (k - level)) break;
        children.push_back(comb);
    }

    if(level == k - 1){

        comb_array.push_back(parent);
        for(vector<uint8_t> child : children){
            comb_array.push_back(child);
        } 

    }
    else{ 

        _increase_elements_after_level(parent, comb_array, n, k, level + 1);
        for(vector<uint8_t> child : children){
            _increase_elements_after_level(child, comb_array, n, k, level + 1);
        } 

    }

}


void anisotropic_diffusion(const Mat &src, Mat &dest, int t_max){
  
  Mat x = src;
  Mat x0;
  x.convertTo(x0, CV_32FC1);

  double t=0;
  double lambda=0.25; // Defined in equation (7)
  double K=10,K2=(1/K/K); // defined after equation(13) in text

  Mat    dI00 = Mat::zeros(x0.size(),CV_32F);

  Mat x1, xc;

  while (t < t_max){

      Mat D; // defined just before equation (5) in text
      Mat gradxX,gradyX; // Image Gradient t time 
      Sobel(x0,gradxX,CV_32F,1,0,3);
      Sobel(x0,gradyX,CV_32F,0,1,3);
      D = Mat::zeros(x0.size(),CV_32F);

      for (int i=0;i<x0.rows;i++)
          for (int j = 0; j < x0.cols; j++)
          {
              float gx = gradxX.at<float>(i, j), gy = gradyX.at<float>(i,j);
              float d;
              if (i==0 || i== x0.rows-1 || j==0 || j==x0.cols-1) // conduction coefficient set to
                  d=1;                                           // 1 p633 after equation 13
              else
                  d =1.0/(1+(gx*gx+0*gy*gy)*K2); // expression of g(gradient(I))
                  //d =-exp(-(gx*gx+gy*gy)*K2); // expression of g(gradient(I))
              D.at<float>(i, j) = d;
         }

      x1 = Mat::zeros(x0.size(),CV_32F);
      double maxD=0,intxx=0;
      {
          int i=0;
          float *u1 = (float*)x1.ptr(i);
          u1++;
          for (int j = 1; j < x0.cols-1; j++,u1++)
              {
                  // Value of I at (i+1,j),(i,j+1)...(i,j)
                  float ip10=x0.at<float>(i+1, j),i0p1=x0.at<float>(i, j+1);
                  float i0m1=x0.at<float>(i, j-1),i00=x0.at<float>(i, j);

                  // Value of D at at (i+1,j),(i,j+1)...(i,j)
                  float cp10=D.at<float>(i+1, j),c0p1=D.at<float>(i, j+1);
                  float c0m1=D.at<float>(i, j-1),c00=D.at<float>(i, j);

                  // Equation (7) p632
                  double xx=(cp10+c00)*(ip10-i00) + (c0p1+c00)*(i0p1-i00) + (c0m1+c00)*(i0m1-i00);
                  dI00.at<float>(i, j) = xx;
                  if (maxD<fabs(xx))
                      maxD=fabs(xx);
                  intxx+=fabs(xx);
                  // equation (9)
             }
      }

      for (int i = 1; i < x0.rows-1; i++)
      {

          float *u1 = (float*)x1.ptr(i);
          int j=0;
          if (j==0)
          {
              // Value of I at (i+1,j),(i,j+1)...(i,j)
              float ip10=x0.at<float>(i+1, j),i0p1=x0.at<float>(i, j+1);
              float im10=x0.at<float>(i-1, j),i00=x0.at<float>(i, j);
              // Value of D at at (i+1,j),(i,j+1)...(i,j)
              float cp10=D.at<float>(i+1, j),c0p1=D.at<float>(i, j+1);
              float cm10=D.at<float>(i-1, j),c00=D.at<float>(i, j);
              // Equation (7) p632
              double xx=(cp10+c00)*(ip10-i00) + (c0p1+c00)*(i0p1-i00) + (cm10+c00)*(im10-i00);
              dI00.at<float>(i, j) = xx;
              if (maxD<fabs(xx))
                  maxD=fabs(xx);
              intxx+=fabs(xx);
              // equation (9)
         }

          u1++;
          j++;
          for (int j = 1; j < x0.cols-1; j++,u1++)
          {
              // Value of I at (i+1,j),(i,j+1)...(i,j)
              float ip10=x0.at<float>(i+1, j),i0p1=x0.at<float>(i, j+1);
              float im10=x0.at<float>(i-1, j),i0m1=x0.at<float>(i, j-1),i00=x0.at<float>(i, j);
              // Value of D at at (i+1,j),(i,j+1)...(i,j)
              float cp10=D.at<float>(i+1, j),c0p1=D.at<float>(i, j+1);
              float cm10=D.at<float>(i-1, j),c0m1=D.at<float>(i, j-1),c00=D.at<float>(i, j);
              // Equation (7) p632
              double xx=(cp10+c00)*(ip10-i00) + (c0p1+c00)*(i0p1-i00) + (cm10+c00)*(im10-i00)+ (c0m1+c00)*(i0m1-i00);
              dI00.at<float>(i, j) = xx;
              if (maxD<fabs(xx))
                  maxD=fabs(xx);
              intxx+=fabs(xx);
              // equation (9)
          }

          j++;
          if (j==x0.cols-1)
          {
              // Value of I at (i+1,j),(i,j+1)...(i,j)
              float ip10=x0.at<float>(i+1, j);
              float im10=x0.at<float>(i-1, j),i0m1=x0.at<float>(i, j-1),i00=x0.at<float>(i, j);

              // Value of D at at (i+1,j),(i,j+1)...(i,j)
              float cp10=D.at<float>(i+1, j);
              float cm10=D.at<float>(i-1, j),c0m1=D.at<float>(i, j-1),c00=D.at<float>(i, j);

              // Equation (7) p632
              double xx=(cp10+c00)*(ip10-i00)  + (cm10+c00)*(im10-i00)+ (c0m1+c00)*(i0m1-i00);
              dI00.at<float>(i, j) = xx;
              if (maxD<fabs(xx))
                  maxD=fabs(xx);
              intxx+=fabs(xx);
              // equation (9)
         }
      }
      {
          int i=x0.rows-1;
          float *u1 = (float*)x1.ptr(i);
          u1++;
          for (int j = 1; j < x0.cols-1; j++,u1++)
          {
              // Value of I at (i+1,j),(i,j+1)...(i,j)
              float i0p1=x0.at<float>(i, j+1);
              float im10=x0.at<float>(i-1, j),i0m1=x0.at<float>(i, j-1),i00=x0.at<float>(i, j);

              // Value of D at at (i+1,j),(i,j+1)...(i,j)
              float c0p1=D.at<float>(i, j+1);
              float cm10=D.at<float>(i-1, j),c0m1=D.at<float>(i, j-1),c00=D.at<float>(i, j);

              // Equation (7) p632
              double xx= (c0p1+c00)*(i0p1-i00) + (cm10+c00)*(im10-i00)+ (c0m1+c00)*(i0m1-i00);
              dI00.at<float>(i, j) = xx;
              if (maxD<fabs(xx))
                  maxD=fabs(xx);
              intxx+=fabs(xx);
              // equation (9)
         }
      }
      lambda=100/maxD;
      // cout <<" lambda = "<< lambda<<"\t Maxd"<<maxD << "\t"<<intxx<<"\n";
      for (int i = 0; i < x0.rows; i++)
      {
          float *u1 = (float*)x1.ptr(i);
          for (int j = 0; j < x0.cols; j++,u1++)
          {
              *u1 = x0.at<float>(i, j) + lambda/4*dI00.at<float>(i, j);
              // equation (9)
         }
      }

      x1.copyTo(x0);
      x0.convertTo(xc,CV_8U);
      t=t+lambda;
  }

  dest = xc.clone();

}


void best_plane_from_combination(const vector<Eigen::Vector3d> &point_list, 
                                 double distance_threshold,
                                 vector<double> &result_coeffs){
  /*
    Takes in a list of 3D points as input. From this list of points, all possible
    combinations of 3 points are determined. For each possible 3 point combination,
    the number of points that lie within a threshold distane of the plane defined 
    is calculated. The combination with the largest number of included points is 
    selected as the best plane. The output is the set of coefficients for the 
    selected plane.
  */
  
  // Calculate indeces for all possible point triplets
  size_t features3D_count = point_list.size();
  vector< vector<uint8_t> > combination_idxs;
  combinations(features3D_count, 3, combination_idxs);

  vector<int> num_included(combination_idxs.size(), 0);

  // Calc the average distances of all 3D features to each possible plane
  // defined by a triplet of 3D features
  vector<double> median_dist_to_plane(combination_idxs.size(), 0);
  vector< vector<double> > planes;
  for(size_t i = 0; i < combination_idxs.size(); i++){

    // Calculate plane equation coefficients from triplet of 3D points
    vector<uint8_t> triplet_idxs = combination_idxs[i];
    vector<double> plane_coeffs(4, 0);
    Eigen::Vector3d pt_1 = point_list[triplet_idxs[0]];
    Eigen::Vector3d pt_2 = point_list[triplet_idxs[1]];
    Eigen::Vector3d pt_3 = point_list[triplet_idxs[2]];
    calc_plane_coeffs(pt_1, pt_2, pt_3, plane_coeffs);
    planes.push_back(plane_coeffs);
    
    vector<double> distances(features3D_count, 0);
    for(size_t j = 0; j < features3D_count; j++){
      double distance = point_to_plane_distance(plane_coeffs[0], plane_coeffs[1],
                                                plane_coeffs[2], plane_coeffs[3],
                                                point_list[j]);
      if(distance > distance_threshold) num_included[i] += 1;
    }

  }

  // Find the triplet with the most included points within the threshold
  int min_idx = std::distance(num_included.begin(),
                              max_element(num_included.begin(),
                                          num_included.end())
                             );
  result_coeffs = planes[min_idx];
}


void calc_plane_coeffs(Eigen::Vector3d &pt1, Eigen::Vector3d &pt2, Eigen::Vector3d &pt3,
                       vector<double> &plane_coeffs){
  /*
    Returns the coefficients of the plane defined by 3 points
    The coefficients are for the equation for a 3D plane expressed as:
    ax + by + dz + d = 0
  */
  
  Eigen::Matrix3f A;
  Eigen::Vector3f b_vec;

  A <<  pt1[0], pt1[1], pt1[2],
        pt2[0], pt2[1], pt2[2],
        pt3[0], pt3[1], pt3[2];
  b_vec << 1, 1, 1;
  // cout << "Here is the matrix A:\n" << A << endl;
  // cout << "Here is the vector b:\n" << b_vec << endl;
  Eigen::Vector3f x = A.colPivHouseholderQr().solve(b_vec);
  // cout << "The solution is:\n" << x << endl;
  double a, b, c, d;
  a = 1;
  b = x[1] / x[0];
  c = x[2] / x[0];
  d = -1 / x[0];
  
  plane_coeffs[0] = a;
  plane_coeffs[1] = b;
  plane_coeffs[2] = c;
  plane_coeffs[3] = d;
}


double point_to_plane_distance(double a, double b, double c, double d, Eigen::Vector3d pt){
  /*
    Returns the shortest distance from an arbitrary point(pt) to an arbitrary plane
    defined by the equation ax + by + dz + d = 0
  */

  double x = pt[0];
  double y = pt[1];
  double z = pt[2];
  double numerator = a*x + b*y + c*z + d;
  numerator = sqrt(numerator * numerator);
  double denominator = sqrt(a*a + b*b + c*c);
  return numerator / denominator;
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
