#include <string>
#include <vector>
#include <iostream>
#include <pcl/point_types.h>
#include <pcl/console/time.h>  // TicToc
#include <pcl/registration/icp.h>
#include <pcl/common/transforms.h>
#include <pcl/common/time.h>
#include <pcl/console/print.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/PointIndices.h>
#include <pcl/conversions.h>
#include <pcl/PCLPointCloud2.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl_conversions/pcl_conversions.h>

#include <string>
#include <algorithm>
#include <opencv2/opencv.hpp>
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
#include <image_geometry/pinhole_camera_model.h>

#include <tf/transform_listener.h>
#include <sensor_msgs/image_encodings.h>

#include <eigen_conversions/eigen_msg.h>
#include <sub8_pcl/sub8_pcl.hpp>
#include "geometry_msgs/PoseStamped.h"
#include "ros/ros.h"

// For stack-tracing on seg-fault
#define BACKWARD_HAS_BFD 1
#include <sub8_build_tools/backward.hpp>

#define VISUALIZE

// ROS_NAMESPACE=/stereo/left rosrun image_proc image_proc
// rosbag play ./holding_buoy_mil.bag -r 0.1
// [1]
// http://docs.ros.org/hydro/api/image_geometry/html/c++/classimage__geometry_1_1PinholeCameraModel.html
// [2] http://docs.pointclouds.org/trunk/classpcl_1_1visualization_1_1_p_c_l_visualizer.html

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

  ros::Timer compute_timer;
  ros::Subscriber data_sub;
  ros::NodeHandle nh;
  ros::ServiceServer service;

  image_transport::CameraSubscriber image_sub;
  image_transport::ImageTransport image_transport;
  image_geometry::PinholeCameraModel cam_model;

  // Visualize
  boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer;

  sub::PointCloudT::Ptr current_cloud;
  bool got_cloud, line_added, computing;
};

Sub8BuoyDetector::Sub8BuoyDetector()
    : viewer(new pcl::visualization::PCLVisualizer("Incoming Cloud")), image_transport(nh) {
  pcl::console::print_highlight("Initializing PCL SLAM\n");
  // Perform match computations
  compute_timer = nh.createTimer(ros::Duration(0.05), &Sub8BuoyDetector::compute_loop, this);
  image_sub = image_transport.subscribeCamera("stereo/left/image_rect_color", 1,
                                              &Sub8BuoyDetector::image_callback, this);

  viewer->addCoordinateSystem(1.0);
  got_cloud = false;
  line_added = false;
  computing = false;

  pcl::console::print_highlight("--PCL SLAM Initialized\n");
  data_sub = nh.subscribe("/stereo/points2", 1, &Sub8BuoyDetector::cloud_callback, this);
}

Sub8BuoyDetector::~Sub8BuoyDetector() { viewer->close(); }

void Sub8BuoyDetector::compute_loop(const ros::TimerEvent &timer_event) { viewer->spinOnce(); }

void Sub8BuoyDetector::image_callback(const sensor_msgs::ImageConstPtr &image_msg,
                                      const sensor_msgs::CameraInfoConstPtr &info_msg) {
  computing = true;
  pcl::console::print_highlight("Getting image\n");
  cv::Mat image_raw;
  cv::Mat image_hsv;
  cv::Mat image_thresh;

  cv_bridge::CvImagePtr input_bridge;
  try {
    input_bridge = cv_bridge::toCvCopy(image_msg, sensor_msgs::image_encodings::BGR8);
    image_raw = input_bridge->image;
  } catch (cv_bridge::Exception &ex) {
    ROS_ERROR("[draw_frames] Failed to convert image");
    return;
  }
  cam_model.fromCameraInfo(info_msg);
  // cv::cvtColor(image_raw, image_hsv, CV_BGR2HSV);
  // believe it or not, this is on purpose (So blue replaces red in HSV)
  cv::cvtColor(image_raw, image_hsv, CV_RGB2HSV);

  // cv::Mat image_rect;
  // rectifyImage (image_raw, image_rect);

  // Compute by inverting K --> line (ray) in homogeneous
  if (!got_cloud) {
    return;
  }
  cv::Point2d pt_cv_2d(250, 250);
  sub::PointXYZT pcl_pt;
  pcl_pt = sub::project_uv_to_cloud(*current_cloud, pt_cv_2d, cam_model);

#ifdef VISUALIZE
  if (!line_added) {
    // viewer->addLine(sub::PointXYZT(0.0, 0.0, 0.0), pcl_pt, 255.0, 255.0, 0.0, "line", 0);
    line_added = true;
  } else {
    // viewer->removeShape("line", 0);
    viewer->removeAllShapes(0);
    pcl::console::print_highlight("Drawing line\n");

    // viewer->addLine(sub::PointXYZT(0.0, 0.0, 0.0), pcl_pt, 255.0, 255.0, 0.0, "line", 0);
    // addCube (const Eigen::Vector3f &translation, const Eigen::Quaternionf &rotation, double
    // width, double height, double depth, const std::string &id="cube", int viewport=0)
  }
#endif
  cv::Point2d cv_uv = cam_model.project3dToPixel(cv::Point3f(pcl_pt.x, pcl_pt.y, pcl_pt.z));
  // cv::circle(image_raw, cv_uv, 12.0, cv::Scalar(0, 255, 0), -1);

  // cv::threshold(image_raw, image_thresh, threshold_value, max_BINARY_value, threshold_type);
  // cv::inRange(image_hsv, cv::Scalar(0, 135, 135), cv::Scalar(20, 255, 255), image_thresh);

  // rgb to flip blue
  cv::inRange(image_hsv, cv::Scalar(105, 135, 135), cv::Scalar(120, 255, 255), image_thresh);
  std::vector<std::vector<cv::Point> > contours;
  std::vector<cv::Vec4i> hierarchy;
  cv::findContours(image_thresh.clone(), contours, hierarchy, CV_RETR_EXTERNAL,
                   CV_CHAIN_APPROX_SIMPLE, cv::Point(0, 0));

  for (size_t i = 0; i < contours.size(); i++) {
    cv::Moments m = cv::moments(contours[i], true);
    cv::Point center(m.m10 / m.m00, m.m01 / m.m00);
    std::vector<cv::Point> approx;
    cv::approxPolyDP(contours[i], approx, 5, true);
    double area = cv::contourArea(approx);
    if (area < 100) {
      continue;
    }
    cv::drawContours(image_raw, contours, i, cv::Scalar(255, 255, 0), -1, 8, hierarchy, 0,
                     cv::Point());

    cv::Rect rect = cv::boundingRect(approx);
    double max_r = std::max(rect.x / 1000, rect.y / 1000);
    cv::circle(image_raw, center, 6.0, cv::Scalar(0, 255, 0), -1);

    sub::PointXYZT centroid_projected = sub::project_uv_to_cloud(*current_cloud, center, cam_model);
    // viewer->addLine(sub::PointXYZT(0.0, 0.0, 0.0), centroid_projected, 255.0, 255.0, 0.0,
    // "line" + boost::to_string(i), 0);
    cv::Point3d pt_cv;
    pt_cv = cam_model.projectPixelTo3dRay(center);
    sub::PointXYZT pt1;
    pt1.x = pt_cv.x * 3;
    pt1.y = pt_cv.y * 3;
    pt1.z = pt_cv.z * 3;

    sub::PointXYZT pt2(0.0, 0.0, 0.0);

    std::string title = "line" + boost::to_string(i);
    std::cout << pt1 << std::endl;
    viewer->addLine(pt1, pt2, 255.0, 255.0, 0.0, title, 0);

    viewer->addCube(
        Eigen::Vector3f(centroid_projected.x, centroid_projected.y, centroid_projected.z),
        Eigen::Quaternionf(0.0, 0.0, 0.0, 1.0), 0.3, 0.3, 0.3, "cube" + boost::to_string(i));
  }

  // cv::imshow("input", image_thresh);
  cv::imshow("input", image_raw);
  cv::waitKey(50);
  computing = false;
}

void Sub8BuoyDetector::cloud_callback(const sensor_msgs::PointCloud2::ConstPtr &input_cloud) {
  if (computing) {
    return;
  }
  sub::PointCloudT::Ptr scene(new sub::PointCloudT());
  sub::PointCloudT::Ptr scene_buffer(new sub::PointCloudT());

  pcl::fromROSMsg(*input_cloud, *scene);

  // Add to pcl_tools
  pcl::VoxelGrid<sub::PointXYZT> voxel_filter;
  voxel_filter.setInputCloud(scene);
  voxel_filter.setLeafSize(0.05f, 0.05f, 0.05f);
  voxel_filter.filter(*scene_buffer);

  // Add to pcl_tools
  // pcl::console::print_highlight("Downsampling\n");
  pcl::StatisticalOutlierRemoval<sub::PointXYZT> outlier_remover;
  outlier_remover.setInputCloud(scene_buffer);
  outlier_remover.setMeanK(20);
  outlier_remover.setStddevMulThresh(0.05);
  current_cloud.reset(new sub::PointCloudT());
  outlier_remover.filter(*current_cloud);

#ifdef VISUALIZE
  if (!got_cloud) {
    pcl::console::print_highlight("Getting new\n");
    viewer->addPointCloud(current_cloud, "current_input");
  } else {
    // pcl::console::print_highlight("Updating\n");
    viewer->updatePointCloud(current_cloud, "current_input");
    viewer->spinOnce();
    // Downsample
  }
#endif

  got_cloud = true;
}

int main(int argc, char **argv) {
  ros::init(argc, argv, "pcl_slam");
  Sub8BuoyDetector *sub8_buoys(new Sub8BuoyDetector());
  ros::spin();
}
