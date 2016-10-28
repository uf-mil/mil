#pragma once

#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/point_types.h>
#include <string>


namespace nav{

class PcdSubPubAlgorithm{
  /*
    virtual base class for algorithms that subscribe to point cloud ROS topics,
    operate on the clouds and publish output clouds to a different topic
  */
  using PCD2 = sensor_msgs::PointCloud2;
  using PointCloud = pcl::PointCloud<pcl::PointXYZ>;

public:
  // Constructors and Destructors
  PcdSubPubAlgorithm(ros::NodeHandle nh, std::string input_pcd_topic, std::string output_pcd_topic);

  // Check status methods

  bool activated()
  {
    return _active;
  }

  bool ok()
  {
    return _ok;
  }

  // Set status methods

  void switchActivation()
  {
    _active = !_active;
  }

  
protected:
  virtual void cloud_cb(const PointCloud::ConstPtr &cloud_msg ) = 0;  // runs algorithm pipeline when a new pcd msg is received

  // Subscribing and storing input
  ros::NodeHandle _nh;
  std::string _input_pcd_topic;
  ros::Subscriber _cloud_sub;
  PCD2 _input_pcd2;
  
  // Storing result and publishing
  std::string _output_pcd_topic;
  ros::Publisher _cloud_pub;
  PCD2 _output_pcd2;

  // Activation status
  bool _active = false;

  // Error flag
  bool _ok = false;
  std::string _err_msg;
};

}