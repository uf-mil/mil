#include <sub8_perception/start_gate.hpp>
Sub8StartGateDetector::Sub8StartGateDetector()
  : nh("~")
  , image_transport_(nh)
  , left_cam_stream_(nh, 1)
  , right_cam_stream_(nh, 1)
  , active_(false)
  , sync_thresh_(0.5)
  , timeout_for_found_(2)
  , tf_listener_(tf_buffer_)
{
  std::string img_topic_left_default = "/camera/front/left/image_rect_color";
  std::string img_topic_right_default = "/camera/front/right/image_rect_color";

  std::string left = nh.param<std::string>("input_left", img_topic_left_default);
  std::string right = nh.param<std::string>("input_right", img_topic_right_default);

  left_cam_stream_.init(img_topic_left_default);
  right_cam_stream_.init(img_topic_right_default);

  marker_pub_ = nh.advertise<visualization_msgs::Marker>("visualization_marker", 1, true);
  vision_request_service_ =
      nh.advertiseService("/vision/start_gate/pose", &Sub8StartGateDetector::vision_request_cb, this);
  active_service_ =
      nh.advertiseService("/vision/start_gate/enable", &Sub8StartGateDetector::set_active_enable_cb, this);

  debug_image_pub_left_ = image_transport_.advertise("left", 1, true);
  debug_image_pub_right_ = image_transport_.advertise("right", 1, true);
  debug_image_pub_canny_ = image_transport_.advertise("canny", 1, true);

  canny_low_ = nh.param<int>("canny_low_", 100);
  canny_ratio_ = nh.param<int>("canny_ratio_", 3.0);
  blur_size_ = nh.param<int>("blur_size_", 1);
  dilate_amount_ = nh.param<int>("dilate_amount_", 3);

  // process 10 images per second
  dt_ = 1.0 / 10.0;

  // Initialize kalman
  // position (x,y,z) and its 1st and 2nd dervatives, euler angles and the 1st and 2nd dervatives
  n_states_ = 18;
  // postion xyz and euler roll, pitch, yaw
  n_measurements_ = 6;

  init_kalman_filter();

  run();
}

bool Sub8StartGateDetector::set_active_enable_cb(std_srvs::SetBool::Request &req, std_srvs::SetBool::Response &res)
{
  active_ = req.data;
  res.success = true;
  return true;
}

bool Sub8StartGateDetector::vision_request_cb(sub8_msgs::VisionRequest::Request &req,
                                              sub8_msgs::VisionRequest::Response &resp)
{
  if (!gate_found_)
  {
    resp.found = false;
    return true;
  }
  Eigen::Affine3d gate_pose(Eigen::Translation3d(gate_position_(0, 0), gate_position_(1, 0), gate_position_(2, 0)) *
                            gate_orientation_);
  Eigen::Affine3d gate_in_map;
  tf2::doTransform(gate_pose, gate_in_map, transform_to_map_);
  resp.pose.pose = tf2::toMsg(gate_in_map);
  resp.found = true;
  return true;
}

void Sub8StartGateDetector::run()
{
  ros::Rate loop_rate(1 / dt_);  // process images 10 times per second
  while (ros::ok())
  {
    if (active_)
      determine_start_gate_position();
    loop_rate.sleep();
    ros::spinOnce();
  }
  return;
}

void Sub8StartGateDetector::determine_start_gate_position()
{
  reset_filter_from_time();

  if (!left_cam_stream_.ok() || !right_cam_stream_.ok())
  {
    ROS_WARN("Start Gate Detector: not getting images.");
    return;
  }

  if (left_cam_stream_.size() < 1 || right_cam_stream_.size() < 1)
  {
    ROS_WARN("Calling too soon -- no images ready");
    return;
  }

  cv::Mat current_image_left, current_image_right;
  current_image_left = left_cam_stream_[0]->image();
  current_image_right = right_cam_stream_[0]->image();

  // Enforce approximate image synchronization
  double left_stamp, right_stamp;
  left_stamp = left_cam_stream_[0]->stamp().toSec();
  right_stamp = right_cam_stream_[0]->stamp().toSec();
  double sync_error = fabs(left_stamp - right_stamp);
  if (sync_error > sync_thresh_)
  {
    ROS_WARN("Left and right not synchronized");
    return;
  }
  try
  {
    transform_to_map_ = tf_buffer_.lookupTransform("map", "front_stereo", ros::Time(0));
  }
  catch (tf2::TransformException &ex)
  {
    ROS_ERROR("No transform from front_left_cam to map");
    return;
  }

  // Filter the image and get a canny image
  cv::Mat processed_image_right = process_image(current_image_right);
  cv::Mat processed_image_left = process_image(current_image_left);

  // Find contours
  std::vector<std::vector<cv::Point>> contours_left, contours_right;
  std::vector<cv::Vec4i> hierarchy_left, hierarchy_right;
  cv::findContours(processed_image_left, contours_left, hierarchy_left, cv::RETR_TREE, cv::CHAIN_APPROX_SIMPLE,
                   cv::Point(-1, -1));
  cv::findContours(processed_image_right, contours_right, hierarchy_right, cv::RETR_TREE, cv::CHAIN_APPROX_SIMPLE,
                   cv::Point(-1, -1));

  // Search for that gate and find the points that define the gate
  std::vector<cv::Point> features_l_t = contour_to_2d_features(contours_left);
  std::vector<cv::Point> features_r_t = contour_to_2d_features(contours_right);

  // Ignore if we don't get enough feature points for the gate
  if (features_l_t.size() < 5 || features_r_t.size() < 5)
    return;

  // Get 4 corner points from the feature points
  std::vector<cv::Point> features_l, features_r;
  features_l = get_corner_center_points(features_l_t);
  features_r = get_corner_center_points(features_r_t);

  // Get the indicies from the right camera features that correspond with the left camera. IE: left_features[i] =
  // right_features[correspondence[i]]
  std::vector<int> correspondence_pair_idxs =
      shortest_pair_stereo_matching(features_l, features_r, current_image_left.rows * 0.02);

  // Ignore if we don't find a good match between camera
  if (std::count(correspondence_pair_idxs.begin(), correspondence_pair_idxs.end(), -1) != 0)
    return;

  std::vector<cv::Scalar> colors = { cv::Scalar(255, 0, 0), cv::Scalar(0, 255, 0), cv::Scalar(0, 0, 255),
                                     cv::Scalar(255, 255, 255) };
  for (size_t i = 0; i < features_l.size(); ++i)
  {
    cv::circle(current_image_left, features_l[i], 10, colors[i], -1);
    cv::circle(current_image_right, features_r[correspondence_pair_idxs[i]], 10, colors[i], -1);
  }

  // Get camera projection matrices
  cv::Matx34d left_cam_mat = left_cam_stream_[0]->getCameraModelPtr()->fullProjectionMatrix();
  cv::Matx34d right_cam_mat = right_cam_stream_[0]->getCameraModelPtr()->fullProjectionMatrix();

  // Calculate 3D stereo reconstructions
  std::vector<Eigen::Vector3d> feature_pts_3d;
  Eigen::Vector3d pt_3D;
  for (size_t i = 0; i < correspondence_pair_idxs.size(); i++)
  {
    if (correspondence_pair_idxs[i] == -1)
      continue;
    cv::Point2d pt_L = features_l[i];
    cv::Point2d pt_R = features_r[correspondence_pair_idxs[i]];
    cv::Matx31d pt_L_hom(pt_L.x, pt_L.y, 1);
    cv::Matx31d pt_R_hom(pt_R.x, pt_R.y, 1);
    cv::Mat X_hom = mil_vision::triangulate_Linear_LS(cv::Mat(left_cam_mat), cv::Mat(right_cam_mat), cv::Mat(pt_L_hom),
                                                      cv::Mat(pt_R_hom));
    X_hom = X_hom / X_hom.at<double>(3, 0);
    pt_3D << X_hom.at<double>(0, 0), X_hom.at<double>(1, 0), X_hom.at<double>(2, 0);
    if (pt_3D(2) < 0)
      return;
    if (pt_3D(2) > 5)
      return;
    feature_pts_3d.push_back(pt_3D);
  }

  // Ignore if the volume of the 4 points is not close to 0
  Eigen::Matrix3d matrix_of_vectors;
  {
    Eigen::Vector3d pt0 = feature_pts_3d[0];
    Eigen::Vector3d pt1 = feature_pts_3d[1];
    Eigen::Vector3d pt2 = feature_pts_3d[2];
    Eigen::Vector3d pt3 = feature_pts_3d[3];

    matrix_of_vectors.col(0) = pt1 - pt0;
    matrix_of_vectors.col(1) = pt2 - pt0;
    matrix_of_vectors.col(2) = pt3 - pt0;
  }
  if (abs(matrix_of_vectors.determinant()) * 100 > 30)
    return;

  // Find the best fit plane
  std::vector<double> plane_constants = best_fit_plane_standard(feature_pts_3d);

  // Project points to best fit plane
  std::vector<Eigen::Vector3d> proj_pts;
  Eigen::Vector3d plane_unit_normal;
  plane_unit_normal << plane_constants[0], plane_constants[1], plane_constants[2];
  plane_unit_normal = plane_unit_normal / plane_unit_normal.norm();

  // Reject if z is too far off
  if (fabs(plane_unit_normal(2, 0)) < 0.5)
    return;

  Eigen::Vector3d pt_on_plane;
  pt_on_plane << 0, 0, -plane_constants[3] / plane_constants[2];
  for (uint8_t pt_idx = 0; pt_idx < feature_pts_3d.size(); ++pt_idx)
  {
    Eigen::Vector3d pt = feature_pts_3d[pt_idx];
    Eigen::Vector3d plane_to_pt_vec = pt - pt_on_plane;
    Eigen::Vector3d plane_to_pt_proj_normal = plane_to_pt_vec.dot(plane_unit_normal) * plane_unit_normal;
    Eigen::Vector3d corr_pt = pt - plane_to_pt_proj_normal;
    proj_pts.push_back(corr_pt);
  }
  // Flip the vector to point towards camera if necessary
  plane_unit_normal = plane_unit_normal(2, 0) < 0 ? plane_unit_normal : -plane_unit_normal;

  // Get the center point
  Eigen::Vector3d center_pt;
  for (auto p : proj_pts)
  {
    center_pt(0, 0) += p[0];
    center_pt(1, 0) += p[1];
    center_pt(2, 0) += p[2];
  }
  center_pt(0, 0) /= proj_pts.size();
  center_pt(1, 0) /= proj_pts.size();
  center_pt(2, 0) /= proj_pts.size();

  update_kalman_filter(center_pt, plane_unit_normal);
  last_time_found_ = ros::Time::now();
  gate_found_ = true;

  // Visualize stuff
  visualize_k_gate_normal();
  visualize_3d_reconstruction(feature_pts_3d, left_cam_mat, right_cam_mat, current_image_left, current_image_right);
  visualize_3d_points_rviz(feature_pts_3d, proj_pts);

  sensor_msgs::ImagePtr dbg_img_msg_left =
      cv_bridge::CvImage(std_msgs::Header(), "bgr8", current_image_left).toImageMsg();
  debug_image_pub_left_.publish(dbg_img_msg_left);

  sensor_msgs::ImagePtr dbg_img_msg_right =
      cv_bridge::CvImage(std_msgs::Header(), "bgr8", current_image_right).toImageMsg();
  debug_image_pub_right_.publish(dbg_img_msg_right);
}

std::vector<cv::Point> Sub8StartGateDetector::contour_to_2d_features(std::vector<std::vector<cv::Point>> &contour)
{
  // Loop through all the contours
  std::vector<cv::Point> features;
  for (size_t i = 0; i < contour.size(); ++i)
  {
    // Check if the contour is a gate
    if (valid_contour(contour.at(i)))
    {
      // Match the contour with a polygon
      auto epsilon = 0.01 * cv::arcLength(contour.at(i), true);
      cv::approxPolyDP(contour.at(i), features, epsilon, true);
    }
  }
  return features;
}

std::vector<cv::Point> Sub8StartGateDetector::get_corner_center_points(const std::vector<cv::Point> &features)
{
  // Get all the possible number of combinations of points
  std::vector<std::vector<uint8_t>> id_comb;
  mil_tools::combinations(features.size(), 2, id_comb);

  // Order pairs of points based of shortest distance. store that in id_comb
  std::sort(id_comb.begin(), id_comb.end(),
            [&features](const std::vector<uint8_t> &a, const std::vector<uint8_t> &b) -> bool {
              auto diffx = features[a[0]].x - features[a[1]].x;
              auto diffy = features[a[0]].y - features[a[1]].y;
              auto distanceA = std::sqrt(std::pow(diffx, 2) + std::pow(diffy, 2));

              diffx = features[b[0]].x - features[b[1]].x;
              diffy = features[b[0]].y - features[b[1]].y;
              auto distanceB = std::sqrt(std::pow(diffx, 2) + std::pow(diffy, 2));

              return distanceA < distanceB;
            });

  // Store the center points between the 4 shortest pairs
  std::vector<cv::Point> features_l(4);
  for (size_t i = 0; i < 4; ++i)
  {
    features_l.at(i) = ((features[id_comb[i][0]] + features[id_comb[i][1]]) / 2);
  }
  return features_l;
}

std::vector<int> Sub8StartGateDetector::shortest_pair_stereo_matching(const std::vector<cv::Point> &features_l,
                                                                      const std::vector<cv::Point> &features_r,
                                                                      int y_axis_diff_thresh)
{
  std::vector<int> correspondence_pair_idxs;
  double curr_min_dist, xdiff, ydiff, dist;
  int curr_min_dist_idx;
  for (size_t i = 0; i < features_l.size(); i++)
  {
    curr_min_dist_idx = -1;
    curr_min_dist = 1E6;
    for (size_t j = 0; j < features_r.size(); j++)
    {
      ydiff = features_l[i].y - features_r[j].y;
      if (abs(ydiff) > y_axis_diff_thresh)
        continue;
      xdiff = features_l[i].x - features_r[j].x;

      dist = sqrt(xdiff * xdiff + ydiff * ydiff);
      if (dist < curr_min_dist)
      {
        curr_min_dist = dist;
        curr_min_dist_idx = j;
      }
    }
    correspondence_pair_idxs.push_back(curr_min_dist_idx);
  }
  return correspondence_pair_idxs;
}

std::vector<double> Sub8StartGateDetector::best_fit_plane_standard(const std::vector<Eigen::Vector3d> &feature_pts_3d)
{
  // Calculate best fit plane
  Eigen::Matrix<double, 4, 3> A;
  Eigen::Matrix<double, 4, 1> b_vec;
  A << feature_pts_3d[0][0], feature_pts_3d[0][1], feature_pts_3d[0][2], feature_pts_3d[1][0], feature_pts_3d[1][1],
      feature_pts_3d[1][2], feature_pts_3d[2][0], feature_pts_3d[2][1], feature_pts_3d[2][2], feature_pts_3d[3][0],
      feature_pts_3d[3][1], feature_pts_3d[3][2];
  b_vec << 1, 1, 1, 1;
  Eigen::Matrix<double, 3, 1> x = A.colPivHouseholderQr().solve(b_vec);
  std::vector<double> plane_constants(4);
  plane_constants[0] = 1;            // A
  plane_constants[1] = x[1] / x[0];  // B
  plane_constants[2] = x[2] / x[0];  // C
  plane_constants[3] = -1 / x[0];    // D
  return plane_constants;
}

void Sub8StartGateDetector::visualize_3d_reconstruction(const std::vector<Eigen::Vector3d> &feature_pts_3d,
                                                        cv::Matx34d left_cam_mat, cv::Matx34d right_cam_mat,
                                                        cv::Mat &current_image_left, cv::Mat &current_image_right)
{
  for (size_t i = 0; i < feature_pts_3d.size(); i++)
  {
    Eigen::Vector3d pt = feature_pts_3d[i];
    cv::Matx41d position_hom(pt(0), pt(1), pt(2), 1);
    cv::Matx31d pt_L_2d_hom = left_cam_mat * position_hom;
    cv::Point2d L_center2d(pt_L_2d_hom(0) / pt_L_2d_hom(2), pt_L_2d_hom(1) / pt_L_2d_hom(2));
    cv::Scalar color(255, 0, 255);
    std::stringstream label;
    label << i;
    cv::circle(current_image_left, L_center2d, 5, color, -1);
    cv::putText(current_image_left, label.str(), L_center2d, CV_FONT_HERSHEY_SIMPLEX, 0.0015 * current_image_left.rows,
                cv::Scalar(0, 0, 0), 2);

    cv::Matx31d pt_R_2d_hom = right_cam_mat * position_hom;
    cv::Point2d R_center2d(pt_R_2d_hom(0) / pt_R_2d_hom(2), pt_R_2d_hom(1) / pt_R_2d_hom(2));
    cv::circle(current_image_right, R_center2d, 5, color, -1);
    cv::putText(current_image_right, label.str(), R_center2d, CV_FONT_HERSHEY_SIMPLEX,
                0.0015 * current_image_right.rows, cv::Scalar(0, 0, 0), 2);
  }
}
void Sub8StartGateDetector::visualize_3d_points_rviz(const std::vector<Eigen::Vector3d> &feature_pts_3d,
                                                     const std::vector<Eigen::Vector3d> &proj_pts)
{
  visualization_msgs::Marker points_raw_marker, points_fixed_marker;
  points_raw_marker.header.stamp = points_fixed_marker.header.stamp = ros::Time::now();
  points_raw_marker.type = points_fixed_marker.type = visualization_msgs::Marker::POINTS;
  points_raw_marker.header.frame_id = points_fixed_marker.header.frame_id = "/front_left_cam";

  points_raw_marker.id = 5;
  points_raw_marker.scale.x = 0.2;
  points_raw_marker.scale.y = 0.2;
  points_raw_marker.color.r = 1.0f;
  points_raw_marker.color.a = 1.0;

  points_fixed_marker.id = 0;
  points_fixed_marker.scale.x = 0.2;
  points_fixed_marker.scale.y = 0.2;
  points_fixed_marker.color.g = 1.0f;
  points_fixed_marker.color.a = 1.0;

  for (size_t i = 0; i < feature_pts_3d.size(); i++)
  {
    geometry_msgs::Point p;
    p.x = feature_pts_3d[i][0];
    p.y = feature_pts_3d[i][1];
    p.z = feature_pts_3d[i][2];
    points_raw_marker.points.push_back(p);

    geometry_msgs::Point psd;
    psd.x = proj_pts[i][0];
    psd.y = proj_pts[i][1];
    psd.z = proj_pts[i][2];
    points_fixed_marker.points.push_back(psd);
  }

  marker_pub_.publish(points_fixed_marker);
  marker_pub_.publish(points_raw_marker);
}

void Sub8StartGateDetector::visualize_k_gate_normal()
{
  geometry_msgs::Point center_pt;
  center_pt.x = gate_position_(0, 0);
  center_pt.y = gate_position_(1, 0);
  center_pt.z = gate_position_(2, 0);

  Eigen::Vector3d normal = gate_orientation_ * Eigen::Vector3d(1, 0, 0);

  geometry_msgs::Point sdp_normalvec_ros;
  sdp_normalvec_ros.x = center_pt.x + normal(0, 0);
  sdp_normalvec_ros.y = center_pt.y + normal(1, 0);
  sdp_normalvec_ros.z = center_pt.z + normal(2, 0);

  visualization_msgs::Marker marker_normal;
  marker_normal.header.seq = 0;
  marker_normal.header.stamp = ros::Time::now();
  marker_normal.header.frame_id = "/front_left_cam";
  marker_normal.id = 2222;
  marker_normal.type = visualization_msgs::Marker::ARROW;
  marker_normal.points.push_back(center_pt);
  marker_normal.points.push_back(sdp_normalvec_ros);
  marker_normal.scale.x = 0.1;
  marker_normal.scale.y = 0.5;
  marker_normal.scale.z = 0.5;
  marker_normal.color.a = 1.0;
  marker_normal.color.r = 1.0;
  marker_normal.color.g = 0.0;
  marker_normal.color.b = 1.0;
  marker_pub_.publish(marker_normal);
}

cv::Mat Sub8StartGateDetector::process_image(cv::Mat &image)
{
  cv::Mat kernal = cv::Mat::ones(5, 5, CV_8U);

  cv::Mat lab;
  cv::cvtColor(image, lab, cv::COLOR_BGR2Lab);

  cv::Mat hsv;
  cv::cvtColor(image, hsv, cv::COLOR_BGR2HSV);

  cv::Mat bitwised_image;
  {
    cv::Mat lab_channels[3];
    cv::split(lab, lab_channels);
    cv::Mat hsv_channels[3];
    cv::split(hsv, hsv_channels);
    cv::bitwise_and(lab_channels[1], hsv_channels[0], bitwised_image);
  }

  cv::Mat processed_image;
  cv::blur(bitwised_image, processed_image, cv::Size(blur_size_, blur_size_));

  cv::Mat canny;
  cv::Canny(processed_image, canny, canny_low_, canny_low_ * 3.0);

  sensor_msgs::ImagePtr dbg_img_msg_canny = cv_bridge::CvImage(std_msgs::Header(), "mono8", canny).toImageMsg();
  debug_image_pub_canny_.publish(dbg_img_msg_canny);

  cv::Mat closing;
  cv::morphologyEx(canny, closing, cv::MORPH_CLOSE, kernal);

  cv::Mat dilation;
  cv::dilate(closing, dilation, kernal, cv::Point(), dilate_amount_);

  return dilation;
}

double Sub8StartGateDetector::get_angle(cv::Point a, cv::Point b, cv::Point c)
{
  auto ba = a - b;
  auto bc = c - b;

  auto cos_ang = ba.dot(bc) / cv::norm(ba) * cv::norm(bc);
  auto angle = acos(cos_ang);

  return angle * 180 / CV_PI;
}

bool Sub8StartGateDetector::valid_contour(std::vector<cv::Point> &contour)
{
  if (cv::isContourConvex(contour))
    return false;
  auto area = cv::contourArea(contour);
  if (area < 3000)
    return false;
  if (area > 30000)
    return false;
  auto epsilon = 0.01 * cv::arcLength(contour, true);
  std::vector<cv::Point> approx;
  cv::approxPolyDP(contour, approx, epsilon, true);
  cv::Moments mu = cv::moments(contour, false);
  auto center = cv::Point(mu.m10 / mu.m00, mu.m01 / mu.m00);

  // Is the center of mass within the contour?
  if (cv::pointPolygonTest(contour, center, false) == 1)
    return false;

  if (approx.size() < 4)
    return false;
  if (approx.size() > 15)
    return false;

  for (size_t i = 0; i < approx.size(); i += 2)
  {
    auto angle = get_angle(approx[i], approx[i + 1], approx[i + 2]);
    if (abs(angle - 90) > 30)
      return false;
  }
  auto angle = get_angle(approx[approx.size() - 1], approx[0], approx[1]);
  if (abs(angle - 90) > 30)
    return false;
  return true;
}

void Sub8StartGateDetector::init_kalman_filter()
{
  k_filter_.init(n_states_, n_measurements_, 0, CV_64F);
  cv::setIdentity(k_filter_.processNoiseCov, cv::Scalar::all(1e-5));
  // How much to trust measurements
  cv::setIdentity(k_filter_.measurementNoiseCov, cv::Scalar::all(0.005));
  cv::setIdentity(k_filter_.errorCovPost, cv::Scalar::all(1));

  // position
  k_filter_.transitionMatrix.at<double>(0, 3) = dt_;
  k_filter_.transitionMatrix.at<double>(1, 4) = dt_;
  k_filter_.transitionMatrix.at<double>(2, 5) = dt_;
  k_filter_.transitionMatrix.at<double>(3, 6) = dt_;
  k_filter_.transitionMatrix.at<double>(4, 7) = dt_;
  k_filter_.transitionMatrix.at<double>(5, 8) = dt_;
  k_filter_.transitionMatrix.at<double>(0, 6) = 0.5 * pow(dt_, 2);
  k_filter_.transitionMatrix.at<double>(1, 7) = 0.5 * pow(dt_, 2);
  k_filter_.transitionMatrix.at<double>(2, 8) = 0.5 * pow(dt_, 2);

  // orientation
  k_filter_.transitionMatrix.at<double>(9, 12) = dt_;
  k_filter_.transitionMatrix.at<double>(10, 13) = dt_;
  k_filter_.transitionMatrix.at<double>(11, 14) = dt_;
  k_filter_.transitionMatrix.at<double>(12, 15) = dt_;
  k_filter_.transitionMatrix.at<double>(13, 16) = dt_;
  k_filter_.transitionMatrix.at<double>(14, 17) = dt_;
  k_filter_.transitionMatrix.at<double>(9, 15) = 0.5 * pow(dt_, 2);
  k_filter_.transitionMatrix.at<double>(10, 16) = 0.5 * pow(dt_, 2);
  k_filter_.transitionMatrix.at<double>(11, 17) = 0.5 * pow(dt_, 2);

  // x
  k_filter_.measurementMatrix.at<double>(0, 0) = 1;
  // y
  k_filter_.measurementMatrix.at<double>(1, 1) = 1;
  // z
  k_filter_.measurementMatrix.at<double>(2, 2) = 1;
  // roll
  k_filter_.measurementMatrix.at<double>(3, 9) = 1;
  // pitch
  k_filter_.measurementMatrix.at<double>(4, 10) = 1;
  // yaw
  k_filter_.measurementMatrix.at<double>(5, 11) = 1;
}

cv::Mat Sub8StartGateDetector::get_measurement_as_cv_mat(Eigen::Vector3d center_point, Eigen::Vector3d normal_vector)
{
  cv::Mat measurement(n_measurements_, 1, CV_64F);
  Eigen::Vector3d euler = Eigen::Quaterniond()
                              .setFromTwoVectors(Eigen::Vector3d(1, 0, 0), normal_vector)
                              .toRotationMatrix()
                              .eulerAngles(0, 1, 2);
  measurement.at<double>(0) = center_point(0, 0);
  measurement.at<double>(1) = center_point(1, 0);
  measurement.at<double>(2) = center_point(2, 0);
  measurement.at<double>(3) = euler(0, 0);
  measurement.at<double>(4) = euler(1, 0);
  measurement.at<double>(5) = euler(2, 0);
  return measurement;
}

void Sub8StartGateDetector::update_kalman_filter(Eigen::Vector3d center_point, Eigen::Vector3d normal_vector)
{
  cv::Mat prediction = k_filter_.predict();
  cv::Mat measurement = get_measurement_as_cv_mat(center_point, normal_vector);

  cv::Mat estimated = k_filter_.correct(measurement);

  this->gate_position_(0, 0) = estimated.at<double>(0);
  this->gate_position_(1, 0) = estimated.at<double>(1);
  this->gate_position_(2, 0) = estimated.at<double>(2);

  Eigen::Vector3d euler;
  euler(0, 0) = estimated.at<double>(9);
  euler(1, 0) = estimated.at<double>(10);
  euler(2, 0) = estimated.at<double>(11);

  Eigen::AngleAxisd rollAngle(euler(0, 0), Eigen::Vector3d::UnitX());
  Eigen::AngleAxisd yawAngle(euler(1, 0), Eigen::Vector3d::UnitY());
  Eigen::AngleAxisd pitchAngle(euler(2, 0), Eigen::Vector3d::UnitZ());

  this->gate_orientation_ = rollAngle * yawAngle * pitchAngle;
}

void Sub8StartGateDetector::reset_filter_from_time()
{
  if (!gate_found_)
    return;
  if (ros::Time::now() - last_time_found_ > timeout_for_found_)
  {
    init_kalman_filter();
    gate_found_ = false;
  }
}