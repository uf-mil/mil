#pragma once
#include <string>
#include <vector>
#include <utility>
#include <iostream>
#include <algorithm>
#include <stdexcept>
#include <cstdint>
#include <iterator>
#include <ctime>

#include <stdlib.h>     //for using the function sleep

#include <boost/thread.hpp>
#include <boost/bind.hpp>
#include <boost/date_time/posix_time/posix_time.hpp>

#include <opencv2/opencv.hpp>
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <Eigen/Dense>
#include <eigen_conversions/eigen_msg.h>
#include <Eigen/StdVector>

#include <ros/ros.h>
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
#include <image_geometry/pinhole_camera_model.h>
#include <tf/transform_listener.h>
#include <sensor_msgs/image_encodings.h>

#include <navigator_vision_lib/cv_tools.hpp>
#include <navigator_vision_lib/visualization.hpp>
#include <sub8_msgs/TorpBoardPoseRequest.h>
#include <sub8_msgs/TBDetectionSwitch.h>

#include "model.h"
#include "stereomodelfitter.h"


// #define SEGMENTATION_DEBUG

#ifdef SEGMENTATION_DEBUG
#warning(Compiling with segmentation debugging info enabled)
#else
#warning(Compiling with NO segmentation debugging info)
#endif

/*
  Warning:
  Because of its multithreadedness, this class cannot be copy constructed.
  For example, the following will not compile:
    ScanTheCodeDetector tb_detector = ScanTheCodeDetector();
  Do this instead:
    ScanTheCodeDetector tb_detector();
*/

class ScanTheCodeDetector
{

public:
    ScanTheCodeDetector();
    ~ScanTheCodeDetector();

    // Public Variables
    double image_proc_scale, feature_min_distance;
    int diffusion_time, max_features, feature_block_size;


private:
    StereoModelFitter* model_fitter = NULL;

    //ATTRIBUTES:

    // ROS
    ros::NodeHandle nh;
    ros::ServiceServer detection_switch;
    ros::ServiceClient pose_client;
    image_transport::CameraSubscriber left_image_sub, right_image_sub;
    image_transport::ImageTransport image_transport;
    image_transport::Publisher debug_image_pub;
    image_geometry::PinholeCameraModel left_cam_model, right_cam_model;
    nav::ImageWithCameraInfo left_most_recent;
    nav::ImageWithCameraInfo right_most_recent;

    // Scan The Code Board detection will be attempted when true
    bool active;

    // Goes into sequential id for pos_est srv request
    long long int run_id;

    // To prevent invalid img pointers from being passed to toCvCopy (segfault)
    boost::mutex left_mtx, right_mtx;

    // RVIZ
    nav::RvizVisualizer rviz;

    // DBG images will be generated and published when true
    bool generate_dbg_img;
    cv::Mat debug_image;
    cv::Rect upper_left, upper_right, lower_left, lower_right;


    //METHODS:

    // Callbacks
    bool detection_activation_switch(
        sub8_msgs::TBDetectionSwitch::Request &req,
        sub8_msgs::TBDetectionSwitch::Response &resp);

    void left_image_callback(const sensor_msgs::ImageConstPtr &image_msg_ptr,
                             const sensor_msgs::CameraInfoConstPtr &info_msg_ptr);
    void
    right_image_callback(const sensor_msgs::ImageConstPtr &image_msg_ptr,
                         const sensor_msgs::CameraInfoConstPtr &info_msg_ptr);

    // Detection / Processing
    void run();
    void process_current_images();
    void init_ros(std::stringstream& log_msg);
    void validate_frame(cv::Mat& current_image_left, cv::Mat& current_image_right, cv::Mat& processing_size_image_left, cv::Mat& processing_size_image_right);



};




