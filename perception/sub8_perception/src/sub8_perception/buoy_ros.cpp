#include <sub8_perception/buoy.hpp>

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
  image_sub = image_transport.subscribeCamera("stereo/right/image_rect_color", 1,
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

  data_sub = nh.subscribe("/stereo/points2", 1, &Sub8BuoyDetector::cloud_callback, this);
  service_3d =
      nh.advertiseService("/vision/buoy/pose", &Sub8BuoyDetector::request_buoy_position, this);
  pcl::console::print_highlight("--PCL Sub8BuoyDetector Initialized\n");
}

Sub8BuoyDetector::~Sub8BuoyDetector() {
#ifdef VISUALIZE
  viewer->close();
#endif
}


void Sub8BuoyDetector::compute_loop(const ros::TimerEvent &timer_event) {
  if (last_draw_image.empty()) {
    // ROS_ERROR("POOP");
    return;
  }
  sensor_msgs::ImagePtr msg;

  msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", last_draw_image).toImageMsg();
  image_pub.publish(msg);

#ifdef VISUALIZE
  viewer->spinOnce();
#endif
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
