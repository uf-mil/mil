#include <sub8_perception/buoy.hpp>

// Compute the buoy position in the camera model frame
// TODO: Check if we have an image
// TODO: Synchronize cloud and image better
bool Sub8BuoyDetector::request_buoy_position(sub8_msgs::VisionRequest::Request &req,
                                             sub8_msgs::VisionRequest::Response &resp) {
  std::string tf_frame;
  Eigen::Vector3f position;
  Eigen::Vector3f filtered_position;
  std::string target_name = req.target_name;
  ROS_ERROR("SERVICE CALL %s", target_name.c_str());

  if (!(color_ranges.count(target_name) > 0)) {
    ROS_ERROR("Requested buoy target (%s) had no color calibration on the parameter server",
              target_name.c_str());
    return false;
  } else if ((!got_cloud) || (!got_image)) {
    // Failure, yo!
    ROS_ERROR("Requested buoy position before we had both image and point cloud data");
    return false;
  }
  computing = true;
  tf_frame = cam_model.tfFrame();

  // Cache the current image
  // cv::Mat target_image = current_image.clone();
  cv::Mat target_image;
  bool got_last_image;
  got_last_image = get_last_image(target_image);
  if (!got_last_image) {
    ROS_ERROR("Could not encode image");
    return false;
  }
  // Filter the cached point cloud
  sub::PointCloudT::Ptr denanned_cleaned_cloud(new sub::PointCloudT());
  sub::PointCloudT::Ptr target_cloud(new sub::PointCloudT());
  sub::PointCloudT::Ptr current_cloud_filtered(new sub::PointCloudT());
  if (target_cloud->points.size() > 1) {
    return false;
  }
  std::vector<int> indices;
  pcl::removeNaNFromPointCloud(*current_cloud, *denanned_cleaned_cloud, indices);
  ROS_ERROR("%d", denanned_cleaned_cloud->size());
  if (denanned_cleaned_cloud->size() == 0) {
    return false;
  }
  sub::voxel_filter<sub::PointXYZT>(denanned_cleaned_cloud, current_cloud_filtered,
                                    0.05f  // leaf size
                                    );

  sub::statistical_outlier_filter<sub::PointXYZT>(current_cloud_filtered, target_cloud,
                                                  20,   // mean k
                                                  0.05  // std_dev threshold
                                                  );
  bool detection_success;
  detection_success =
      determine_buoy_position(cam_model, target_name, target_image, target_cloud, position);
  if (!detection_success) {
    ROS_WARN("Could not detect %s buoy", target_name.c_str());
    return false;
  }

  //// Temporarily dead code -- filters instead of sending raw position estimate
  // if (last_bump_target == Eigen::Vector3f(0.0, 0.0, 0.0)) {
  //   filtered_position = position;
  // } else {
  //   filtered_position = (0.3 * position) + (0.7 * last_bump_target);
  // }
  // last_bump_target = filtered_position;
  // tf::pointEigenToMsg(filtered_position.cast<double>(), resp.pose.pose.position);

  tf::pointEigenToMsg(position.cast<double>(), resp.pose.pose.position);

  resp.pose.header.frame_id = tf_frame;
  resp.pose.header.stamp = last_image_msg->header.stamp;
  rviz.visualize_buoy(resp.pose.pose, tf_frame);

#ifdef VISUALIZE
  cv::Mat draw_image = target_image.clone();
  cv::Point2d cv_pt_uv =
      cam_model.project3dToPixel(cv::Point3f(position.x(), position.y(), position.z()));
  cv::circle(draw_image, cv_pt_uv, 10, cv::Scalar(0, 240, 30), 4);
  cv::imshow("input", draw_image);
  cv::waitKey(50);
#endif
  computing = false;
  resp.found = true;
  return true;
}

// Compute a 3d point on the surface of the closest buoy
//
// @param[in] camera_model An image_geometry pinhole camera model (Must have data!)
// @param[in] image_raw The current image (Must correspond to the point cloud)
// @param[in] point_cloud_raw The current PointCloud (Must correspond to the image)
// @param[out] center An approximate center of the buoy
bool Sub8BuoyDetector::determine_buoy_position(
    const image_geometry::PinholeCameraModel &camera_model, const std::string &target_color,
    const cv::Mat &image_raw, const sub::PointCloudT::Ptr &point_cloud_raw,
    Eigen::Vector3f &center) {
  cv::Mat image_hsv;
  cv::Mat image_thresh;

  // believe it or not, this is on purpose (So blue replaces red in HSV)
  // cv::cvtColor(image_raw, image_hsv, CV_RGB2HSV);
  cv::cvtColor(image_raw, image_hsv, CV_BGR2HSV);

  // Threshold -- > This is what must be replaced with better 2d vision
  sub::inParamRange(image_hsv, color_ranges[target_color], image_thresh);
  std::vector<sub::Contour> contours;
  std::vector<cv::Vec4i> hierarchy;
  cv::findContours(image_thresh.clone(), contours, hierarchy, CV_RETR_EXTERNAL,
                   CV_CHAIN_APPROX_SIMPLE, cv::Point(0, 0));
  // TODO: ^^^ Make into function ^^^

  // Not returning segmented point cloud for now
  // Vector of Eigen-vectors
  std::vector<Eigen::Vector3f, Eigen::aligned_allocator<Eigen::Vector3f> > buoy_centers;

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
    sub::PointXYZT centroid_projected =
        sub::project_uv_to_cloud(*point_cloud_raw, contour_centroid, camera_model);

    Eigen::Vector3f surface_point = sub::point_to_eigen(centroid_projected);
    // Slide the surface point one buoy radius away from the camera to approximate the 3d center
    Eigen::Vector3f approximate_center = surface_point;// + (buoy_radius * surface_point.normalized());

    // Check if the estimated projection onto the point cloud is outside of the contour
    cv::Point2d centroid_deprojected = cam_model.project3dToPixel(
        cv::Point3f(approximate_center.x(), approximate_center.y(), approximate_center.z()));

    // Check if the projected point falls within the contour
    double insideness;
    insideness = cv::pointPolygonTest(contours[i], centroid_deprojected, true);
    // We accept up to 5 px outside of the contour
    if (insideness < -5) {
      continue;
    }

    buoy_centers.push_back(approximate_center);
  }
  // ^^^^ The above could be eliminated with better approach

  if (buoy_centers.size() == 0) {
    return false;
  }

  // TODO: initialize to inf
  // Get the closest
  double closest_distance = 1000;
  size_t closest_buoy_index = 0;
  for (size_t buoy_index = 0; buoy_index < buoy_centers.size(); buoy_index++) {
    const double distance = buoy_centers[buoy_index].norm();
    if (buoy_centers[buoy_index].norm() < closest_distance) {
      closest_distance = distance;
      closest_buoy_index = buoy_index;
    }
  }

  center = buoy_centers[closest_buoy_index];

  return true;
}
