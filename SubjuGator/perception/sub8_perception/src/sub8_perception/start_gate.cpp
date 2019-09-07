#include <sub8_perception/start_gate.hpp>
Sub8StartGateDetector::Sub8StartGateDetector() : nh("~"), timeout_for_found_(2), tf_listener_(tf_buffer_)
{
  // Start the camera streamers
  left_cam_stream_ = std::unique_ptr<ROSCameraStream_Vec3>(new ROSCameraStream_Vec3(nh, 1));
  right_cam_stream_ = std::unique_ptr<ROSCameraStream_Vec3>(new ROSCameraStream_Vec3(nh, 1));

  std::string img_topic_left_default = "/camera/front/left/image_rect_color";
  std::string img_topic_right_default = "/camera/front/right/image_rect_color";

  std::string left = nh.param<std::string>("left_camera_topic", img_topic_left_default);
  std::string right = nh.param<std::string>("right_camera_topic", img_topic_right_default);

  // Initialize the streamers with the topic paths
  left_cam_stream_->init(left);
  right_cam_stream_->init(right);

  canny_low_ = nh.param<int>("canny_low_", 100);
  canny_ratio_ = nh.param<int>("canny_ratio_", 3.0);
  blur_size_ = nh.param<int>("blur_size_", 1);
  dilate_amount_ = nh.param<int>("dilate_amount_", 3);

  // Should node be processing image
  active_ = false;
  // The maximum time difference between the two camera time stamps
  sync_thresh_ = 0.5;
  // process 10 times per second
  refresh_rate_ = 10;

  gate_found_ = false;

  // Visualization
  marker_pub_ = nh.advertise<visualization_msgs::Marker>("visualization_marker", 1, true);

  // Service providers for VisionProxy
  vision_request_service_ =
      nh.advertiseService("/vision/start_gate/pose", &Sub8StartGateDetector::vision_request_cb, this);
  active_service_ =
      nh.advertiseService("/vision/start_gate/enable", &Sub8StartGateDetector::set_active_enable_cb, this);

  run();
}

std::vector<cv::Point> Sub8StartGateDetector::get_2d_feature_points(cv::Mat image)
{
  // Filter the image
  cv::Mat processed_image = process_image(image);

  // Find contours
  std::vector<std::vector<cv::Point>> contours;
  std::vector<cv::Vec4i> hierarchy;
  cv::findContours(processed_image, contours, hierarchy, cv::RETR_TREE, cv::CHAIN_APPROX_SIMPLE, cv::Point(-1, -1));

  // Find contour that closely matches a gate
  std::vector<cv::Point> features = contour_to_2d_features(contours);

  // Ignore if we don't get enough feature points for the gate
  if (features.size() < 5)
    return {};

  return get_corner_center_points(features);
}

void Sub8StartGateDetector::run()
{
  ros::Rate loop_rate(refresh_rate_);
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
  // Find transform between map frame and stereo frame
  try
  {
    transform_to_map_ = tf_buffer_.lookupTransform("map", "front_stereo", ros::Time(0));
  }
  catch (tf2::TransformException &ex)
  {
    ROS_ERROR("No transform from front_left_cam to map");
    return;
  }

  // If no gates have been found in a while, reset kalman
  if (gate_found_ && ros::Time::now() - last_time_found_ > timeout_for_found_)
  {
    init_kalman_filter();
    gate_found_ = false;
  }

  // If cameras are out of sync or not publishing, don't do anything
  if (!is_stereo_coherent())
    return;

  auto feature_pts_3d_ptr = get_3d_feature_points();
  if (!feature_pts_3d_ptr)
    return;
  // Use inherited function to find 3d points and then estimate a pose
  auto pose_ptr = get_3d_pose(*feature_pts_3d_ptr);
  if (pose_ptr)
  {
    auto pose = *pose_ptr;
    gate_pose_ = update_kalman_filter(pose);
    gate_found_ = true;
    last_time_found_ = ros::Time::now();
    visualize_3d_points_rviz(*feature_pts_3d_ptr);
    visualize_k_gate_normal();
  }
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
  Eigen::Affine3d gate_in_map;
  tf2::doTransform(gate_pose_, gate_in_map, transform_to_map_);
  resp.pose.pose = tf2::toMsg(gate_in_map);
  resp.found = true;
  return true;
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

  // sensor_msgs::ImagePtr dbg_img_msg_canny = cv_bridge::CvImage(std_msgs::Header(), "mono8", canny).toImageMsg();
  // debug_image_pub_canny_.publish(dbg_img_msg_canny);

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

void Sub8StartGateDetector::visualize_k_gate_normal()
{
  geometry_msgs::Point center_pt;
  center_pt.x = gate_pose_.translation().x();
  center_pt.y = gate_pose_.translation().y();
  center_pt.z = gate_pose_.translation().z();

  Eigen::Vector3d normal = gate_pose_.rotation() * Eigen::Vector3d(1, 0, 0);

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

void Sub8StartGateDetector::visualize_3d_points_rviz(const std::vector<Eigen::Vector3d> &feature_pts_3d)
{
  visualization_msgs::Marker points_raw_marker;
  points_raw_marker.header.stamp = ros::Time::now();
  points_raw_marker.type = visualization_msgs::Marker::POINTS;
  points_raw_marker.header.frame_id = "/front_left_cam";

  points_raw_marker.id = 5;
  points_raw_marker.scale.x = 0.2;
  points_raw_marker.scale.y = 0.2;
  points_raw_marker.color.r = 1.0f;
  points_raw_marker.color.a = 1.0;

  for (size_t i = 0; i < feature_pts_3d.size(); i++)
  {
    geometry_msgs::Point p;
    p.x = feature_pts_3d[i][0];
    p.y = feature_pts_3d[i][1];
    p.z = feature_pts_3d[i][2];
    points_raw_marker.points.push_back(p);
  }

  marker_pub_.publish(points_raw_marker);
}