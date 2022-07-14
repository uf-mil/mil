#pragma once

#include <dynamic_reconfigure/client.h>
#include <dynamic_reconfigure/server.h>
#include <mil_bounds/BoundsConfig.h>
#include <mil_msgs/ObjectDBQuery.h>
#include <nav_msgs/Odometry.h>
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <std_srvs/Trigger.h>
#include <tf2/convert.h>
#include <tf2_eigen/tf2_eigen.h>
#include <tf2_msgs/TFMessage.h>
#include <tf2_ros/transform_listener.h>

#include <boost/circular_buffer.hpp>

#include "input_cloud_filter.hpp"
#include "marker_manager.hpp"
#include "object_associator.hpp"
#include "object_detector.hpp"
#include "object_map.hpp"
#include "ogrid_manager.hpp"
#include "pcodar_types.hpp"
#include "persistent_cloud_filter.hpp"
#include "point_cloud_builder.hpp"

namespace pcodar
{
/**
 * Base class for a class implementing the object detection needed for the PCODAR node.
 * This can be fulfilled by processesing LIDAR pointclouds (like in @pcodar::Node) or using
 * simulated ground truth (like in pcodar_gazebo).
 */
class NodeBase
{
public:
  /// Create a NodeBase in the namespace of nh
  NodeBase(ros::NodeHandle nh);
  /// Initialize ROS communication
  virtual void initialize();
  /// Update markers, ogrid, and publish the internal object map to ROS interfaces. Call after updating objects.
  void UpdateObjects();

protected:
  /// Process a database query ROS service
  bool DBQuery_cb(mil_msgs::ObjectDBQuery::Request& req, mil_msgs::ObjectDBQuery::Response& res);
  /// Reset PCODAR
  virtual bool Reset(std_srvs::Trigger::Request& req, std_srvs::Trigger::Response& res);
  /// Transform
  bool transform_to_global(std::string const& frame, ros::Time const& time, Eigen::Affine3d& out,
                           ros::Duration timeout = ros::Duration(1, 0));
  /// Transform a pointcloud ROS message into a PCL pointcloud in the global frame
  bool transform_point_cloud(const sensor_msgs::PointCloud2& pcloud2, point_cloud& out);
  virtual bool bounds_update_cb(const mil_bounds::BoundsConfig& config);
  virtual void ConfigCallback(Config const& config, uint32_t level);

public:
  std::shared_ptr<ObjectMap> objects_;

protected:
  ros::NodeHandle nh_;
  dynamic_reconfigure::Client<mil_bounds::BoundsConfig> bounds_client_;
  dynamic_reconfigure::Server<Config> config_server_;

  ros::ServiceServer modify_classification_service_;
  ros::ServiceServer reset_service_;

  std::string global_frame_;

  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener;

  // Publishers
  ros::Publisher pub_objects_;

  point_cloud_ptr bounds_;

  // Visualization
  MarkerManager marker_manager_;
  OgridManager ogrid_manager_;
};

class Node : public NodeBase
{
public:
  Node(ros::NodeHandle nh);

  void velodyne_cb(const sensor_msgs::PointCloud2ConstPtr& pcloud);

  void initialize() override;

private:
  bool bounds_update_cb(const mil_bounds::BoundsConfig& config) override;
  void ConfigCallback(Config const& config, uint32_t level) override;
  /// Reset PCODAR
  bool Reset(std_srvs::Trigger::Request& req, std_srvs::Trigger::Response& res) override;

private:
  ros::Publisher pub_pcl_;

  // Subscriber
  ros::Subscriber pc_sub;

  // Model (It eventually will be obeject tracker, but for now just detections)
  InputCloudFilter input_cloud_filter_;
  PersistentCloudFilter persistent_cloud_filter_;
  PointCloudCircularBuffer persistent_cloud_builder_;
  ObjectDetector detector_;
  Associator ass;
};

}  // namespace pcodar
