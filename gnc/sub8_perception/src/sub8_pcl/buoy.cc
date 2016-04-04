#include <sub8_pcl/buoy.hpp>

Sub8BuoyDetector::Sub8BuoyDetector()
    : vp1(0),
      vp2(1),
#ifdef VISUALIZE
      viewer(new pcl::visualization::PCLVisualizer("Incoming Cloud")),
#endif
      rviz("/visualization/buoys"),
      last_bump_target(0.0, 0.0, 0.0),
      image_transport(nh) {
  pcl::console::print_highlight("Initializing PCL Sub8BuoyDetector\n");

  // Check if radius parameter exists
  // TODO: Make this templated library code, allow defaults
  if (nh.hasParam("vision/buoy_radius")) {
    nh.getParam("vision/buoy_radius", buoy_radius);
  } else {
    buoy_radius = 0.1016;  // m
  }

  compute_timer = nh.createTimer(ros::Duration(0.09), &Sub8BuoyDetector::compute_loop, this);
  image_sub = image_transport.subscribeCamera("stereo/left/image_rect_color", 1,
                                              &Sub8BuoyDetector::image_callback, this);
  image_pub = image_transport.advertise("vision/buoy/target_info", 1);

#ifdef VISUALIZE
  viewer->addCoordinateSystem(1.0);
  viewer->createViewPort(0.5, 0.0, 1.0, 1.0, vp1);
  viewer->createViewPort(0.0, 0.0, 0.5, 1.0, vp2);
#endif

  got_cloud = false;
  line_added = false;
  computing = false;
  need_new_cloud = false;

  // Not yet able to build with C++11, should be done with an initialized vector
  std::string red_topic = "/color/buoy/red";
  sub::range_from_param(red_topic, color_ranges["red"]);
  // TODO: color_ranges["green"] = ...
  // TODO: color_ranges["yellow"] = ...

  data_sub = nh.subscribe("/stereo/points2", 1, &Sub8BuoyDetector::cloud_callback, this);
  service_2d =
      nh.advertiseService("/vision/buoys/2d", &Sub8BuoyDetector::request_buoy_position_2d, this);
  service_3d =
      nh.advertiseService("/vision/buoys/pose", &Sub8BuoyDetector::request_buoy_position, this);
  pcl::console::print_highlight("--PCL Sub8BuoyDetector Initialized\n");
}

Sub8BuoyDetector::~Sub8BuoyDetector() {
#ifdef VISUALIZE
  viewer->close();
#endif
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
  cv::cvtColor(target_image, image_hsv, CV_RGB2HSV);

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
    if (area < 100) {
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
  } else if ((!got_cloud) && (!got_image)) {
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
  sub::PointCloudT::Ptr target_cloud(new sub::PointCloudT());
  sub::PointCloudT::Ptr current_cloud_filtered(new sub::PointCloudT());
  if (target_cloud->points.size() > 1) {
    return false;
  }
  sub::voxel_filter<sub::PointXYZT>(current_cloud, current_cloud_filtered,
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

  return true;
}

void Sub8BuoyDetector::compute_loop(const ros::TimerEvent &timer_event) {
  if (last_draw_image.empty()) {
    ROS_ERROR("POOP");
    return;
  }
  sensor_msgs::ImagePtr msg;

  msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", last_draw_image).toImageMsg();
  image_pub.publish(msg);

#ifdef VISUALIZE
  viewer->spinOnce();
#endif
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
  cv::cvtColor(image_raw, image_hsv, CV_RGB2HSV);

  cv::Point2d pt_cv_2d(250, 250);
  sub::PointXYZT pcl_pt_3d;
  pcl_pt_3d = sub::project_uv_to_cloud(*point_cloud_raw, pt_cv_2d, camera_model);

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
    if (area < 100) {
      continue;
    }
    sub::PointXYZT centroid_projected =
        sub::project_uv_to_cloud(*point_cloud_raw, contour_centroid, camera_model);

    Eigen::Vector3f surface_point = sub::point_to_eigen(centroid_projected);
    // Slide the surface point one buoy radius away from the camera to approximate the 3d center
    Eigen::Vector3f approximate_center = surface_point + (buoy_radius * surface_point.normalized());

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

bool Sub8BuoyDetector::get_last_image(cv::Mat &last_image) {
  cv_bridge::CvImagePtr input_bridge;
  if (!got_image) {
    return false;
  }
  try {
    input_bridge = cv_bridge::toCvCopy(last_image_msg, sensor_msgs::image_encodings::BGR8);
    last_image = input_bridge->image;
  } catch (cv_bridge::Exception &ex) {
    ROS_ERROR("Failed to convert image");
    return false;
  }
  return true;
}

void Sub8BuoyDetector::image_callback(const sensor_msgs::ImageConstPtr &image_msg,
                                      const sensor_msgs::CameraInfoConstPtr &info_msg) {
  need_new_cloud = true;
  got_image = true;

#ifdef VISUALIZE
  pcl::console::print_highlight("Getting image\n");
#endif

  // cam_model message does not change with time, so syncing is not a big deal
  last_image_msg = image_msg;
  cam_model.fromCameraInfo(info_msg);
  image_time = image_msg->header.stamp;
}

void Sub8BuoyDetector::cloud_callback(const sensor_msgs::PointCloud2::ConstPtr &input_cloud) {
  if (computing) {
    return;
  }

  // Require reasonable time-similarity
  // (Not using message filters because image_transport eats the image and info msgs. Is there a
  // better way to do this?)
  if (((input_cloud->header.stamp - image_time) < ros::Duration(0.3)) and (need_new_cloud)) {
    last_cloud_time = input_cloud->header.stamp;
    need_new_cloud = false;
  } else {
    return;
  }

  current_cloud.reset(new sub::PointCloudT());
  pcl::fromROSMsg(*input_cloud, *current_cloud);

#ifdef VISUALIZE
  sub8_msgs::VisionRequest::Request req;
  req.target_name = "red";
  sub8_msgs::VisionRequest::Response resp;
  request_buoy_position(req, resp);
  pcl::console::print_highlight("Getting Point Cloud\n");
  if (!got_cloud) {
    viewer->addPointCloud(current_cloud, "current_input", vp1);
  } else {
    viewer->updatePointCloud(current_cloud, "current_input");
    viewer->spinOnce();
    // Downsample
  }
#endif

  got_cloud = true;
}
