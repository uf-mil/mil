#pragma once

#include <ros/ros.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/point_types.h>
#include <string>


namespace nav{

template <typename input_T, typename output_T>
class PcdSubPubAlgorithm{
  /*
    virtual base class for algorithms that subscribe to point cloud ROS topics,
    operate on the clouds and publish output clouds to a different topic
  */

  template<typename T> using PCD = pcl::PointCloud<T>;

public:

  // Constructors and Destructors

  PcdSubPubAlgorithm(ros::NodeHandle nh, std::string input_pcd_topic, std::string output_pcd_topic)
  : _nh(nh), _input_pcd_topic(input_pcd_topic), _output_pcd_topic(output_pcd_topic)
  {
    // Subscribe to point cloud topic
    _cloud_sub = _nh.subscribe<PCD<input_T>>(_input_pcd_topic, 1, &PcdSubPubAlgorithm::cloud_cb, this);

    // Advertise output topic
    _cloud_pub = _nh.advertise<PCD<output_T>>(_output_pcd_topic, 1, true);
  }

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

  // runs algorithm pipeline when a new pcd msg is received
  virtual void cloud_cb(const PCD<in_PCD_T>::ConstPtr &cloud_msg ) = 0;  // runs algorithm pipeline when a new pcd msg is received

  // Subscribing and storing input
  ros::NodeHandle _nh;
  std::string _input_pcd_topic;
  ros::Subscriber _cloud_sub;
  PCD<input_T> _input_pcd2;
  
  // Storing result and publishing
  std::string _output_pcd_topic;
  ros::Publisher _cloud_pub;
  PCD<output_T> _output_pcd2;

  // Activation status
  bool _active = false;

  // Error flag
  bool _ok = false;
  std::string _err_msg;
};

}  //namespace nav