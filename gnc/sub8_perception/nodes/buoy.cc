#include <string>
#include <vector>
#include <iostream>
#include <pcl/console/time.h>  // TicToc
#include <pcl/common/time.h>
#include <pcl/console/print.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/conversions.h>
#include <pcl/PCLPointCloud2.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl_conversions/pcl_conversions.h>

#include <algorithm>
#include <opencv2/opencv.hpp>
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
#include <image_geometry/pinhole_camera_model.h>

#include <tf/transform_listener.h>
#include <sensor_msgs/image_encodings.h>

#include <eigen_conversions/eigen_msg.h>
#include <Eigen/StdVector>
#include <sub8_pcl/pcl_tools.hpp>
#include <sub8_pcl/cv_tools.hpp>

#include "ros/ros.h"

#include "sub8_msgs/VisionRequest.h"

// For stack-tracing on seg-fault
#define BACKWARD_HAS_BFD 1
#include <sub8_build_tools/backward.hpp>

#define VISUALIZE

// ROS_NAMESPACE=/stereo/left rosrun image_proc image_proc
// rosbag play ./holding_buoy_mil.bag -r 0.1
// [1]
// http://docs.ros.org/hydro/api/image_geometry/html/c++/classimage__geometry_1_1PinholeCameraModel.html
// [2] http://docs.pointclouds.org/trunk/classpcl_1_1visualization_1_1_p_c_l_visualizer.html
/* TODO
  - Smarter buoy tracking
  - Better cacheing (instead of immediate assignment)
*/

bool visualize = false;
std::string meshpath;
class Sub8BuoyDetector {
 public:
  Sub8BuoyDetector();
  ~Sub8BuoyDetector();
  void compute_loop(const ros::TimerEvent &);
  void cloud_callback(const sensor_msgs::PointCloud2::ConstPtr &);
  void image_callback(const sensor_msgs::ImageConstPtr &msg,
                      const sensor_msgs::CameraInfoConstPtr &info_msg);
  void determine_buoy_position(const image_geometry::PinholeCameraModel &cam_model,
                               const cv::Mat &image_raw,
                               const sub::PointCloudT::Ptr &point_cloud_raw,
                               Eigen::Vector3f &center);

  bool request_buoy_position(sub8_msgs::VisionRequest::Request &req,
                             sub8_msgs::VisionRequest::Response &resp);
  // Visualize
  int vp1;
  int vp2;
  boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer;
  sub::RvizVisualizer rviz;

  ros::Timer compute_timer;
  ros::Subscriber data_sub;
  ros::NodeHandle nh;
  ros::ServiceServer service;
  double buoy_radius;

  image_transport::CameraSubscriber image_sub;
  image_transport::ImageTransport image_transport;
  image_geometry::PinholeCameraModel cam_model;

  ros::Time image_time;
  ros::Time last_cloud_time;
  sub::PointCloudT::Ptr current_cloud;
  cv::Mat current_image;
  bool got_cloud, got_image, line_added, computing, need_new_cloud;
};

Sub8BuoyDetector::Sub8BuoyDetector()
    : vp1(0),
      vp2(1),
#ifdef VISUALIZE
      viewer(new pcl::visualization::PCLVisualizer("Incoming Cloud")),
#endif
      image_transport(nh) {
  pcl::console::print_highlight("Initializing PCL SLAM\n");

  // Check if radius parameter exists
  // TODO: Make this templated library code, allow defaults
  if (nh.hasParam("buoy_radius")) {
    nh.getParam("buoy_radius", buoy_radius);
  } else {
    buoy_radius = 0.1016;  // m
  }

  compute_timer = nh.createTimer(ros::Duration(0.09), &Sub8BuoyDetector::compute_loop, this);
  image_sub = image_transport.subscribeCamera("stereo/left/image_rect_color", 1,
                                              &Sub8BuoyDetector::image_callback, this);

#ifdef VISUALIZE
  viewer->addCoordinateSystem(1.0);
  viewer->createViewPort(0.5, 0.0, 1.0, 1.0, vp1);
  viewer->createViewPort(0.0, 0.0, 0.5, 1.0, vp2);
#endif

  got_cloud = false;
  line_added = false;
  computing = false;
  need_new_cloud = false;

  pcl::console::print_highlight("--PCL SLAM Initialized\n");
  data_sub = nh.subscribe("/stereo/points2", 1, &Sub8BuoyDetector::cloud_callback, this);
  service =
      nh.advertiseService("/vision/buoys/red", &Sub8BuoyDetector::request_buoy_position, this);
}

Sub8BuoyDetector::~Sub8BuoyDetector() {
#ifdef VISUALIZE
  viewer->close();
#endif
}

// Compute the buoy position in the camera model frame
// TODO: Check if we have an image
// TODO: Synchronize cloud and image better
bool Sub8BuoyDetector::request_buoy_position(sub8_msgs::VisionRequest::Request &req,
                                             sub8_msgs::VisionRequest::Response &resp) {
  std::string tf_frame;
  Eigen::Vector3f position;

  if ((!got_cloud) && (!got_image)) {
    // Failure, yo!
    ROS_ERROR("Requested buoy position before we had both image and point cloud data");
    return false;
  }
  computing = true;
  tf_frame = cam_model.tfFrame();

  // Cache the current image
  cv::Mat target_image = current_image.clone();

  // Filter the cached point cloud
  sub::PointCloudT::Ptr target_cloud(new sub::PointCloudT());
  sub::PointCloudT::Ptr current_cloud_filtered(new sub::PointCloudT());
  sub::voxel_filter<sub::PointXYZT>(current_cloud, current_cloud_filtered,
                                    0.05f  // leaf size
                                    );

  sub::statistical_outlier_filter<sub::PointXYZT>(current_cloud_filtered, target_cloud,
                                                  20,   // mean k
                                                  0.05  // std_dev threshold
                                                  );

  determine_buoy_position(cam_model, target_image, target_cloud, position);

  tf::pointEigenToMsg(position.cast<double>(), resp.pose.pose.position);
  resp.pose.header.frame_id = tf_frame;
  rviz.visualize_buoy(resp.pose.pose, tf_frame);

#ifdef VISUALIZE
  cv::Point2d cv_pt_uv =
      cam_model.project3dToPixel(cv::Point3f(position.x(), position.y(), position.z()));
  cv::circle(current_image, cv_pt_uv, 7, cv::Scalar(0, 20, 240), 2);
  cv::imshow("input", current_image);
  cv::waitKey(50);
#endif
  computing = false;

  return true;
}

void Sub8BuoyDetector::compute_loop(const ros::TimerEvent &timer_event) {
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
void Sub8BuoyDetector::determine_buoy_position(
    const image_geometry::PinholeCameraModel &camera_model, const cv::Mat &image_raw,
    const sub::PointCloudT::Ptr &point_cloud_raw, Eigen::Vector3f &center) {
  cv::Mat image_hsv;
  cv::Mat image_thresh;

  // believe it or not, this is on purpose (So blue replaces red in HSV)
  cv::cvtColor(image_raw, image_hsv, CV_RGB2HSV);

  cv::Point2d pt_cv_2d(250, 250);
  sub::PointXYZT pcl_pt_3d;
  pcl_pt_3d = sub::project_uv_to_cloud(*point_cloud_raw, pt_cv_2d, camera_model);

  // Reprojection (Useful for visualization)
  cv::Point2d cv_pt_uv =
      cam_model.project3dToPixel(cv::Point3f(pcl_pt_3d.x, pcl_pt_3d.y, pcl_pt_3d.z));

  // Threshold -- > This is what must be replaced with better 2d vision
  cv::inRange(image_hsv, cv::Scalar(105, 135, 135), cv::Scalar(120, 255, 255), image_thresh);
  std::vector<sub::Contour> contours;
  std::vector<cv::Vec4i> hierarchy;
  cv::findContours(image_thresh.clone(), contours, hierarchy, CV_RETR_EXTERNAL,
                   CV_CHAIN_APPROX_SIMPLE, cv::Point(0, 0));
  // TODO: ^^^ Make into function ^^^

  // Not returning segmented point cloud for now
  // sub::PointCloudT::Ptr segmented_cloud(new sub::PointCloudT());
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
    buoy_centers.push_back(approximate_center);
  }
  // ^^^^ The above could be eliminated with better approach

  // TODO: initialize to inf
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
}

void Sub8BuoyDetector::image_callback(const sensor_msgs::ImageConstPtr &image_msg,
                                      const sensor_msgs::CameraInfoConstPtr &info_msg) {
  need_new_cloud = true;
  got_image = true;

#ifdef VISUALIZE
  pcl::console::print_highlight("Getting image\n");
#endif

  cv_bridge::CvImagePtr input_bridge;
  try {
    input_bridge = cv_bridge::toCvCopy(image_msg, sensor_msgs::image_encodings::BGR8);
    current_image = input_bridge->image;
  } catch (cv_bridge::Exception &ex) {
    ROS_ERROR("[draw_frames] Failed to convert image");
    return;
  }
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

  // sub::PointCloudT::Ptr scene(new sub::PointCloudT());
  // sub::PointCloudT::Ptr scene_buffer(new sub::PointCloudT());

  current_cloud.reset(new sub::PointCloudT());
  pcl::fromROSMsg(*input_cloud, *current_cloud);

/*
sub::statistical_outlier_filter<sub::PointXYZT>(scene_buffer, current_cloud,
                                                20,   // mean k
                                                0.05  // std_dev threshold
                                                );
                                                */
#ifdef VISUALIZE
  pcl::console::print_highlight("Getting Point Cloud\n");
  if (!got_cloud) {
    pcl::console::print_highlight("Getting new\n");
    viewer->addPointCloud(current_cloud, "current_input", vp1);
  } else {
    viewer->updatePointCloud(current_cloud, "current_input");
    viewer->spinOnce();
    // Downsample
  }
#endif

  got_cloud = true;
}

int main(int argc, char **argv) {
  ros::init(argc, argv, "pcl_slam");
  boost::shared_ptr<Sub8BuoyDetector> sub8_buoys(new Sub8BuoyDetector());
  ros::spin();
}
