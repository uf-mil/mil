#pragma once
#include <string>
#include <iostream>

#include "ros/ros.h"

#include <image_transport/image_transport.h>
#include <image_geometry/pinhole_camera_model.h>

#include <opencv2/highgui/highgui.hpp>
#include <cv_bridge/cv_bridge.h>

#include <boost/circular_buffer.hpp>
#include <boost/accumulators/accumulators.hpp>
#include <boost/accumulators/statistics.hpp>

#include <std_srvs/SetBool.h>

#include "sub8_msgs/VisionRequest2D.h"

using namespace boost::accumulators;

class Sub8StartGateDetector {
public:
    Sub8StartGateDetector();
    ~Sub8StartGateDetector();

    const double START_GATE_WIDTH = 2.0; //Meters
    const double START_GATE_HEIGHT = 1.0;

    void image_callback(const sensor_msgs::ImageConstPtr &msg,
                        const sensor_msgs::CameraInfoConstPtr &info_msg);
    bool request_start_gate_position_2d(sub8_msgs::VisionRequest2D::Request &req,
                                        sub8_msgs::VisionRequest2D::Response &resp);
    bool request_start_gate_enable(std_srvs::SetBool::Request &req,
                                   std_srvs::SetBool::Response &resp);
    void findGate(const sensor_msgs::ImageConstPtr &image_msg);

    double distance();

    ros::NodeHandle nh;
    bool running;

    image_transport::ImageTransport image_transport;
    image_transport::CameraSubscriber image_sub;

    image_geometry::PinholeCameraModel cam_model;
    ros::Time image_time;

    int rows;
    int cols;

    boost::circular_buffer<std::vector<std::vector<cv::Point2f>>> gate_line_buffer;
    accumulator_set<int, features<tag::mean, tag::variance>> accX;
    accumulator_set<int, features<tag::mean, tag::variance>> accY;
    accumulator_set<int, features<tag::mean, tag::variance>> accSizeX;
    accumulator_set<int, features<tag::mean, tag::variance>> accSizeY;




    ros::ServiceServer service_enable;
    ros::ServiceServer service_2d;
};