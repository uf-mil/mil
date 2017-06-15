#include <sub8_perception/start_gate.hpp>
Sub8StartGateDetector::Sub8StartGateDetector() : nh("~"), image_transport_(nh), active_(true), sync_thresh_(0.5)
{
  std::string img_topic_left_default = "/camera/front/left/image_rect_color";
  std::string img_topic_right_default = "/camera/front/right/image_rect_color";

  std::string left = nh.param<std::string>("~input_left", img_topic_left_default);
  std::string right = nh.param<std::string>("~input_right", img_topic_right_default);
  left_image_sub_ = image_transport_.subscribeCamera(left, 10, &Sub8StartGateDetector::left_image_callback, this);
  right_image_sub_ = image_transport_.subscribeCamera(right, 10, &Sub8StartGateDetector::right_image_callback, this);

  marker_pub_ = nh.advertise<visualization_msgs::Marker>("~visualization_marker", 1, true);
  center_gate_pub_ = nh.advertise<geometry_msgs::Point>("~center", 1, true);
  normal_gate_pub_ = nh.advertise<geometry_msgs::Vector3>("~normal", 1, true);
  active_service_ = nh.advertiseService("~enable", &Sub8StartGateDetector::set_active_enable_cb, this);

  debug_image_pub_left_ = image_transport_.advertise("~left", 1, true);
  debug_image_pub_right_ = image_transport_.advertise("~right", 1, true);
  debug_image_pub_canny_ = image_transport_.advertise("~canny", 1, true);

  canny_low_ = nh.param<int>("~canny_low_", 100);
  canny_ratio_ = nh.param<int>("~canny_ratio_", 3.0);
  blur_size_ = nh.param<int>("~blur_size_", 1);
  dilate_amount_ = nh.param<int>("~dilate_amount_", 3);

  run();
}

void Sub8StartGateDetector::left_image_callback(const sensor_msgs::ImageConstPtr &image_msg_ptr,
                                                const sensor_msgs::CameraInfoConstPtr &info_msg_ptr)
{
  left_mtx_.lock();
  left_most_recent.image_msg_ptr = image_msg_ptr;
  left_most_recent.info_msg_ptr = info_msg_ptr;
  left_mtx_.unlock();
}

void Sub8StartGateDetector::right_image_callback(const sensor_msgs::ImageConstPtr &image_msg_ptr,
                                                 const sensor_msgs::CameraInfoConstPtr &info_msg_ptr)
{
  right_mtx_.lock();
  right_most_recent.image_msg_ptr = image_msg_ptr;
  right_most_recent.info_msg_ptr = info_msg_ptr;
  right_mtx_.unlock();
}

bool Sub8StartGateDetector::set_active_enable_cb(std_srvs::SetBool::Request &req, std_srvs::SetBool::Response &res)
{
  active_ = req.data;
  res.success = true;
  return true;
}

void Sub8StartGateDetector::run()
{
  ros::Rate loop_rate(10);  // process images 10 times per second
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
  // Prevent segfault if service is called before we get valid img_msg_ptr's
  if (left_most_recent.image_msg_ptr == NULL || right_most_recent.image_msg_ptr == NULL)
  {
    ROS_WARN("Start Gate Detector: Image Pointers are NULL.");
    return;
  }

  cv_bridge::CvImagePtr input_bridge;
  cv::Mat current_image_left, current_image_right, processing_size_image_left, processing_size_image_right;

  try
  {
    left_mtx_.lock();
    right_mtx_.lock();

    // Left Camera
    input_bridge = cv_bridge::toCvCopy(left_most_recent.image_msg_ptr, sensor_msgs::image_encodings::BGR8);
    current_image_left = input_bridge->image;
    left_cam_model_.fromCameraInfo(left_most_recent.info_msg_ptr);
    if (current_image_left.channels() != 3)
    {
      ROS_ERROR("The left image topic does not contain a color image.");
      return;
    }

    // Right Camera
    input_bridge = cv_bridge::toCvCopy(right_most_recent.image_msg_ptr, sensor_msgs::image_encodings::BGR8);
    current_image_right = input_bridge->image;
    right_cam_model_.fromCameraInfo(right_most_recent.info_msg_ptr);
    if (current_image_right.channels() != 3)
    {
      ROS_ERROR("The right image topic does not contain a color image.");
      return;
    }
    left_mtx_.unlock();
    right_mtx_.unlock();
  }
  catch (const std::exception &ex)
  {
    ROS_ERROR("[start_gate] cv_bridge: Failed to convert images");
    left_mtx_.unlock();
    right_mtx_.unlock();
    return;
  }

  // Enforce approximate image synchronization
  double left_stamp, right_stamp;
  left_stamp = left_most_recent.image_msg_ptr->header.stamp.toSec();
  right_stamp = right_most_recent.image_msg_ptr->header.stamp.toSec();
  double sync_error = fabs(left_stamp - right_stamp);
  if (sync_error > sync_thresh_)
  {
    ROS_WARN("Left and right not synchronized");
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
  cv::Matx34d left_cam_mat = left_cam_model_.fullProjectionMatrix();
  cv::Matx34d right_cam_mat = right_cam_model_.fullProjectionMatrix();

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

  geometry_msgs::Point center_pt;
  for (auto p : proj_pts)
  {
    center_pt.x += p[0];
    center_pt.y += p[1];
    center_pt.z += p[2];
  }
  center_pt.x /= proj_pts.size();
  center_pt.y /= proj_pts.size();
  center_pt.z /= proj_pts.size();

  geometry_msgs::Point sdp_normalvec_ros;
  sdp_normalvec_ros.x = center_pt.x + plane_unit_normal(0, 0);
  sdp_normalvec_ros.y = center_pt.y + plane_unit_normal(1, 0);
  sdp_normalvec_ros.z = center_pt.z + plane_unit_normal(2, 0);
  geometry_msgs::Vector3 normal_vec;
  normal_vec.x = plane_unit_normal(0, 0);
  normal_vec.y = plane_unit_normal(1, 0);
  normal_vec.z = plane_unit_normal(2, 0);

  // TODO: kalman filter for pose
  // TODO: make this a vision proxy
  center_gate_pub_.publish(center_pt);
  normal_gate_pub_.publish(normal_vec);

  // Visualize stuff
  visualize_3d_reconstruction(feature_pts_3d, left_cam_mat, right_cam_mat, current_image_left, current_image_right);
  visualize_3d_points_rviz(feature_pts_3d, proj_pts);

  visualization_msgs::Marker marker_normal;
  marker_normal.header.seq = 0;
  marker_normal.header.stamp = ros::Time::now();
  marker_normal.header.frame_id = "/front_left_cam";
  marker_normal.id = 3000;
  marker_normal.type = visualization_msgs::Marker::ARROW;
  marker_normal.points.push_back(center_pt);
  marker_normal.points.push_back(sdp_normalvec_ros);
  marker_normal.scale.x = 0.1;
  marker_normal.scale.y = 0.5;
  marker_normal.scale.z = 0.5;
  marker_normal.color.a = 1.0;
  marker_normal.color.r = 0.0;
  marker_normal.color.g = 0.0;
  marker_normal.color.b = 1.0;
  marker_pub_.publish(marker_normal);

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
  combinations(features.size(), 2, id_comb);

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

void Sub8StartGateDetector::combinations(uint8_t n, uint8_t k, std::vector<std::vector<uint8_t>> &idx_array)
{
  idx_array = std::vector<std::vector<uint8_t>>();

  std::vector<uint8_t> first_comb;

  // set first combination indices
  for (uint8_t i = 0; i < k; i++)
  {
    first_comb.push_back(i);
  }

  uint8_t level = 0;

  _increase_elements_after_level(first_comb, idx_array, n, k, level);
}

void Sub8StartGateDetector::_increase_elements_after_level(std::vector<uint8_t> comb,
                                                           std::vector<std::vector<uint8_t>> &comb_array, uint8_t n,
                                                           uint8_t k, uint8_t level)
{
  std::vector<uint8_t> parent = comb;
  std::vector<std::vector<uint8_t>> children;

  while (true)
  {
    for (uint8_t idx = level; idx < k; idx++)
    {
      comb[idx] = comb[idx] + 1;
    }
    if (comb[level] > n - (k - level))
      break;
    children.push_back(comb);
  }

  if (level == k - 1)
  {
    comb_array.push_back(parent);
    for (std::vector<uint8_t> child : children)
    {
      comb_array.push_back(child);
    }
  }
  else
  {
    _increase_elements_after_level(parent, comb_array, n, k, level + 1);
    for (std::vector<uint8_t> child : children)
    {
      _increase_elements_after_level(child, comb_array, n, k, level + 1);
    }
  }
}
