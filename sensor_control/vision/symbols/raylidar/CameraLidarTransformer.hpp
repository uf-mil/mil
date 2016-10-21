#pragma once
#include <ros/ros.h>
// PCL specific includes
#include <geometry_msgs/TransformStamped.h>
#include <geometry_msgs/Vector3Stamped.h>
#include <image_geometry/pinhole_camera_model.h>
#include <message_filters/cache.h>
#include <message_filters/subscriber.h>
#include <navigator_msgs/CameraToLidarTransform.h>
#include <pcl/point_cloud.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/point_types.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl_ros/point_cloud.h>
#include <sensor_msgs/CameraInfo.h>
#include <sensor_msgs/PointCloud2.h>
#include <tf2/convert.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2_msgs/TFMessage.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_sensor_msgs/tf2_sensor_msgs.h>
#include <visualization_msgs/MarkerArray.h>
#include <boost/accumulators/accumulators.hpp>
#include <boost/accumulators/statistics.hpp>
#include <iostream>

#define DO_ROS_DEBUG
#ifdef DO_ROS_DEBUG
#include "opencv2/opencv.hpp"
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
#include <sensor_msgs/image_encodings.h>
#endif

class CameraLidarTransformer
{
  private:
    union floatConverter
    {
        float f;
        struct
        {
            uint8_t data[4];
        };
    };
    std::string camera_info_topic;
    // ~double MIN_Z,MAX_Z_ERROR;
    ros::NodeHandle nh;
    ros::ServiceServer transformServiceServer;
    tf2_ros::Buffer tfBuffer;
    tf2_ros::TransformListener tfListener;
    message_filters::Subscriber<sensor_msgs::PointCloud2> lidarSub;
    message_filters::Cache<sensor_msgs::PointCloud2> lidarCache;
    ros::Subscriber cameraInfoSub;
    image_geometry::PinholeCameraModel cam_model;
    sensor_msgs::CameraInfo camera_info;
    ros::Publisher pubMarkers;
    void cameraInfoCallback(const sensor_msgs::CameraInfo info);
    bool transformServiceCallback(navigator_msgs::CameraToLidarTransform::Request &req,navigator_msgs::CameraToLidarTransform::Response &res);
#ifdef DO_ROS_DEBUG
    image_transport::ImageTransport image_transport;
    image_transport::Publisher points_debug_publisher;
#endif
  public:
      CameraLidarTransformer();
};
