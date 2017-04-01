#include <sub8_perception/buoy.hpp>

bool Sub8BuoyDetector::segment_buoy(cv::Mat &input_image, cv::Point &center,
                                    std::vector<sub::Contour> &output_contours, std::string &target_name) {
  cv::Mat image_hsv;
  cv::Mat image_thresh;

  // believe it or not, this is on purpose (So blue replaces red in HSV)
  cv::cvtColor(input_image, image_hsv, CV_BGR2HSV);

  // Threshold -- > This is what must be replaced with better 2d vision
  sub::inParamRange(image_hsv, color_ranges[target_name], image_thresh);
  std::vector<cv::Vec4i> hierarchy;
  std::vector<sub::Contour> contours;
  cv::findContours(image_thresh.clone(), contours, hierarchy, CV_RETR_EXTERNAL,
                   CV_CHAIN_APPROX_SIMPLE, cv::Point(0, 0));

  cv::Point best_centroid;
  std::vector<cv::Point> buoy_centers;
  double max_area = 0;
  // Loop through the contours we found and find the positions
  for (size_t i = 0; i < contours.size(); i++) {
    cv::Point contour_centroid = sub::contour_centroid(contours[i]);
    std::vector<cv::Point> approx;

    // Magic num: 5
    cv::approxPolyDP(contours[i], approx, 5, true);
    double area = cv::contourArea(approx);

    // Ignore small regions
    if (area < 20) {
      continue;
    }

    if (area > max_area) {
      best_centroid = contour_centroid;
      max_area = area;
    }

    buoy_centers.push_back(contour_centroid);
    output_contours.push_back(contours[i]);
  }

  if (buoy_centers.size() == 0) {
    // return false;
    return true;
  }
  // center = buoy_centers[closest_buoy_index];
  center = best_centroid;
  last_draw_image = input_image.clone();
  cv::circle(last_draw_image, best_centroid, 10, cv::Scalar(0, 240, 30), 4);
  return true;
}

bool Sub8BuoyDetector::request_buoy_position_2d(sub8_msgs::VisionRequest2D::Request &req,
                                                sub8_msgs::VisionRequest2D::Response &resp) {
  std::string tf_frame;
  Eigen::Vector3f position;
  Eigen::Vector3f filtered_position;
  std::string target_name = req.target_name;

  if (!(color_ranges.count(target_name) > 0)) {
    ROS_ERROR("Requested buoy target (%s) had no color calibration on the parameter server",
              target_name.c_str());
    return false;
  } else if ((!got_cloud) && (!got_image)) {
    // Failure, yo!
    ROS_ERROR("Requested buoy position before we had both image and point cloud data");
    return false;
  }

  computing = true;
  tf_frame = cam_model.tfFrame();

  // Cache the current image
  cv::Mat target_image;
  if (!get_last_image(target_image)) {
    ROS_ERROR("Could not encode image");
    return false;
  }

  cv::Mat image_hsv;
  cv::Mat image_thresh;

  // believe it or not, this is on purpose (So blue replaces red in HSV)
  // cv::cvtColor(target_image, image_hsv, CV_RGB2HSV);
  cv::cvtColor(target_image, image_hsv, CV_BGR2HSV);


  // Threshold -- > This is what must be replaced with better 2d vision
  sub::inParamRange(image_hsv, color_ranges[target_name], image_thresh);
  std::vector<sub::Contour> contours;
  std::vector<cv::Vec4i> hierarchy;
  cv::findContours(image_thresh.clone(), contours, hierarchy, CV_RETR_EXTERNAL,
                   CV_CHAIN_APPROX_SIMPLE, cv::Point(0, 0));

  cv::Point best_centroid;
  std::vector<cv::Point> buoy_centers;
  double max_area = 0;
  // Loop through the contours we found and find the positions
  for (size_t i = 0; i < contours.size(); i++) {
    cv::Point contour_centroid = sub::contour_centroid(contours[i]);
    std::vector<cv::Point> approx;

    // Magic num: 5
    cv::approxPolyDP(contours[i], approx, 5, true);
    double area = cv::contourArea(approx);

    // Ignore small regions
    if (area < 20) {
      continue;
    }

    if (area > max_area) {
      best_centroid = contour_centroid;
      max_area = area;
    }
    buoy_centers.push_back(contour_centroid);
  }

  if (buoy_centers.size() == 0) {
    // return false;
    resp.found = false;
    return true;
  }
  // center = buoy_centers[closest_buoy_index];
  last_draw_image = target_image.clone();
  cv::circle(last_draw_image, best_centroid, 10, cv::Scalar(0, 240, 30), 4);
  resp.pose.x = best_centroid.x;
  resp.pose.y = best_centroid.y;
  resp.pose.theta = 0.0;

  resp.found = true;
  return true;
}
