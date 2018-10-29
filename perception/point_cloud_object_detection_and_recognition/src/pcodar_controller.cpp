#include <pcl/point_cloud.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl_ros/point_cloud.h>
#include <pcl_ros/transforms.h>
#include <functional>
#include <point_cloud_object_detection_and_recognition/pcodar_controller.hpp>

namespace pcodar
{
NodeBase::NodeBase(ros::NodeHandle _nh)
  : nh_(_nh)
  , bounds_client_("/bounds_server", std::bind(&NodeBase::bounds_update_cb, this, std::placeholders::_1))
  , tf_listener(tf_buffer_, nh_)
  , global_frame_("enu")
  , config_server_(_nh)
{
  config_server_.setCallback(std::bind(&NodeBase::ConfigCallback, this, std::placeholders::_1, std::placeholders::_2));
}

void NodeBase::ConfigCallback(Config const& config, uint32_t level)
{
  if (!level || level & 16)
    ogrid_manager_.update_config(config);
}

void NodeBase::UpdateObjects()
{
  ogrid_manager_.update_ogrid(objects_);
  auto objects_msg = objects_.to_msg();
  marker_manager_.update_markers(objects_msg);
  pub_objects_.publish(objects_msg);
}

void NodeBase::initialize()
{
  marker_manager_.initialize(nh_);
  ogrid_manager_.initialize(nh_);

  modify_classification_service_ = nh_.advertiseService("/database/requests", &NodeBase::DBQuery_cb, this);

  // Publish PerceptionObjects
  pub_objects_ = nh_.advertise<mil_msgs::PerceptionObjectArray>("objects", 1);
}

bool NodeBase::transform_to_global(std::string const& frame, ros::Time const& time, Eigen::Affine3d& out,
                                   ros::Duration timeout)
{
  geometry_msgs::TransformStamped transform;
  try
  {
    transform = tf_buffer_.lookupTransform("enu", frame, time, timeout);
  }
  catch (tf2::TransformException& ex)
  {
    ROS_ERROR("%s", ex.what());
    return false;
  }
  out = tf2::transformToEigen(transform);
  return true;
}

bool NodeBase::bounds_update_cb(const mil_bounds::BoundsConfig& config)
{
  Eigen::Affine3d transform;
  if (!transform_to_global(config.frame, ros::Time::now(), transform, ros::Duration(10)))
    return false;

  bounds_ = boost::make_shared<point_cloud>();
  bounds_->push_back(point_t(config.x1, config.y1, config.z1));
  bounds_->push_back(point_t(config.x2, config.y2, config.z2));
  bounds_->push_back(point_t(config.x3, config.y3, config.z3));
  bounds_->push_back(point_t(config.x4, config.y4, config.z4));

  pcl::transformPointCloud(*bounds_, *bounds_, transform);
  ogrid_manager_.set_bounds(bounds_);

  return true;
}

bool NodeBase::DBQuery_cb(mil_msgs::ObjectDBQuery::Request& req, mil_msgs::ObjectDBQuery::Response& res)
{
  return objects_.DatabaseQuery(req, res);
}

bool NodeBase::transform_point_cloud(const sensor_msgs::PointCloud2& pc_msg, point_cloud& out)
{
  Eigen::Affine3d transform;
  if (!transform_to_global(pc_msg.header.frame_id, pc_msg.header.stamp, transform))
    return false;

  // Transform from PCL2
  pcl::PCLPointCloud2 pcl_pc2;
  pcl_conversions::toPCL(pc_msg, pcl_pc2);
  point_cloud pcloud;
  pcl::fromPCLPointCloud2(pcl_pc2, pcloud);

  out.clear();
  pcl::transformPointCloud(pcloud, out, transform);
  return true;
}

Node::Node(ros::NodeHandle _nh) : NodeBase(_nh)
{
  config_server_.setCallback(std::bind(&Node::ConfigCallback, this, std::placeholders::_1, std::placeholders::_2));

  // TODO: pull from params
  point_cloud robot_footprint;
  robot_footprint.push_back(point_t(2.4384, 1.2192, 0.));
  robot_footprint.push_back(point_t(2.4384, -1.2192, 0.));
  robot_footprint.push_back(point_t(-2.4384, -1.2192, 0.));
  robot_footprint.push_back(point_t(-2.4384, 1.2192, 0.));
  // Give the filter the footprint of the robot to remove from pointcloud
  input_cloud_filter_.set_robot_footprint(robot_footprint);
}

void Node::ConfigCallback(Config const& config, uint32_t level)
{
  NodeBase::ConfigCallback(config, level);
  if (!level || level & 1)
    persistent_cloud_builder_.update_config(config);
  if (!level || level & 2)
    persistent_cloud_filter_.update_config(config);
  if (!level || level & 4)
    detector_.update_config(config);
  if (!level || level & 8)
    ass.update_config(config);
}

void Node::initialize()
{
  NodeBase::initialize();

  // Subscribe pointcloud
  pc_sub = nh_.subscribe("/velodyne_points", 1, &Node::velodyne_cb, this);

  // Publish occupancy grid and visualization markers
  pub_pcl_ = nh_.advertise<point_cloud>("persist_pcl", 1);
}

void Node::velodyne_cb(const sensor_msgs::PointCloud2ConstPtr& pcloud)
{
  point_cloud_ptr pc = boost::make_shared<point_cloud>();
  // Transform new pointcloud to ENU
  if (!transform_point_cloud(*pcloud, *pc))
    return;

  // Get current pose of robot to filter neaby points
  Eigen::Affine3d robot_transform;
  if (!transform_to_global("base_link", pcloud->header.stamp, robot_transform))
    return;
  input_cloud_filter_.set_robot_pose(robot_transform);

  // Filter out bounds / robot
  point_cloud_ptr filtered_pc = boost::make_shared<point_cloud>();
  input_cloud_filter_.filter(pc, *filtered_pc);

  // Add pointcloud to persistent cloud
  persistent_cloud_builder_.add_point_cloud(filtered_pc);

  // Get persistent cloud and publish for debug
  auto accrued = persistent_cloud_builder_.get_point_cloud();
  if ((*accrued).empty())
    return;

  // Filter out outliers
  point_cloud_ptr filtered_accrued = boost::make_shared<point_cloud>();
  persistent_cloud_filter_.filter(accrued, *filtered_accrued);

  // Publish accrued cloud
  (*filtered_accrued).header.frame_id = "enu";
  pub_pcl_.publish(filtered_accrued);

  // Get object clusters from persistent pointcloud
  clusters_t clusters = detector_.get_clusters(filtered_accrued);

  // Associate current clusters with old ones
  ass.associate(objects_, *filtered_accrued, clusters);

  UpdateObjects();
}

bool Node::bounds_update_cb(const mil_bounds::BoundsConfig& config)
{
  if (!NodeBase::bounds_update_cb(config))
    return false;
  input_cloud_filter_.set_bounds(bounds_);
  return true;
}

}  // namespace pcodar
