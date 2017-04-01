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

  if ((!got_cloud) || (!got_image)) {
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

  if (denanned_cleaned_cloud->size() == 0) {
    return false;
  }

  // Nothing clean in a genocide
  target_cloud = denanned_cleaned_cloud->makeShared();
  // sub::voxel_filter<sub::PointXYZT>(denanned_cleaned_cloud, current_cloud_filtered,
  //                                   0.05f  // leaf size
  //                                   );

  //sub::statistical_outlier_filter<sub::PointXYZT>(current_cloud_filtered, target_cloud,
  //                                                20,   // mean k
  //                                                0.05  // std_dev threshold
  //                                                );

  bool detection_success;
  detection_success =
      determine_buoy_position(cam_model, target_name, target_image, target_cloud, position);
  if (!detection_success) {
    ROS_WARN("Could not detect %s buoy", target_name.c_str());
    return false;
  }

  tf::pointEigenToMsg(position.cast<double>(), resp.pose.pose.position);

  resp.pose.header.frame_id = tf_frame;
  resp.pose.header.stamp = last_image_msg->header.stamp;
  rviz.visualize_buoy(resp.pose.pose, tf_frame);

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


  sub8_msgs::VisionRequest2D::Request req;
  sub8_msgs::VisionRequest2D::Response resp;

  req.target_name = target_color;
  bool got_good_response = ros::service::call("vision/buoy/2D", req, resp);
  if (!got_good_response) {
    ROS_ERROR("Got bad response when requesting 2d position");
    return false;
  }

  cv::Point contour_centroid(resp.pose.x, resp.pose.y);
  std::cout << resp.pose.x << " " << resp.pose.y << "zzz" << std::endl;

  double distance;
  sub::PointXYZT centroid_projected =
      sub::project_uv_to_cloud(*point_cloud_raw, contour_centroid, camera_model, distance);

  Eigen::Vector3f surface_point = sub::point_to_eigen(centroid_projected);
  // Slide the surface point one buoy radius away from the camera to approximate the 3d center
  Eigen::Vector3f approximate_center = surface_point;// + (buoy_radius * surface_point.normalized());

  // if (distance > (buoy_radius * 1.3)) {
  //   return false;
  // }

  center = approximate_center;

  return true;
}
