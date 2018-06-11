#include "OGridGen.hpp"

// TODO: Add service call to clear ogrid

ogrid_param params;

OGridGen::OGridGen()
  : nh_(ros::this_node::getName())
  , kill_listener_(nh_, "kill")
  , was_killed_(true)
  , classification_(&nh_)
  , pointCloud_(new pcl::PointCloud<pcl::PointXYZI>())
{
  // The publishers
  pub_grid_ = nh_.advertise<nav_msgs::OccupancyGrid>("ogrid", 10, true);
  pub_point_cloud_filtered_ = nh_.advertise<pcl::PointCloud<pcl::PointXYZI>>("point_cloud/filtered", 1);
  pub_point_cloud_raw_ = nh_.advertise<pcl::PointCloud<pcl::PointXYZI>>("point_cloud/raw", 1);
  pub_point_cloud_plane_ = nh_.advertise<pcl::PointCloud<pcl::PointXYZI>>("point_cloud/plane", 1);
  pub_markers_ = nh_.advertise<visualization_msgs::MarkerArray>("markers", 1);
  pub_objects_ = nh_.advertise<mil_msgs::PerceptionObjectArray>("objects", 1);
  clear_ogrid_service_ = nh_.advertiseService("clear_ogrid", &OGridGen::clear_ogrid_callback, this);
  clear_pcl_service_ = nh_.advertiseService("clear_pcl", &OGridGen::clear_pcl_callback, this);
  get_objects_service_ = nh_.advertiseService("get_objects", &OGridGen::get_objects_callback, this);
  // Do ogrid?
  nh_.param<bool>("ogrid", params.ogrid, false);
  // Resolution is meters/pixel
  nh_.param<float>("resolution", resolution_, 0.2f);
  nh_.param<float>("ogrid_size", ogrid_size_, 91.44);
  // Ignore points that are below the potential pool
  nh_.param<int>("min_intensity", min_intensity_, 2000);
  nh_.param<float>("statistical_mean_k", params.statistical_mean_k, 75);
  nh_.param<float>("statistical_stddev_mul_thresh", params.statistical_stddev_mul_thresh, .75);
  nh_.param<float>("cluster_tolerance_m", params.cluster_tolerance_m, 5);
  nh_.param<float>("cluster_min_num_points", params.cluster_min_num_points, 5);
  nh_.param<float>("cluster_max_num_points", params.cluster_max_num_points, 100);
  nh_.param<float>("nearby_threshold", params.nearby_threshold, 1);
  nh_.param<float>("depth", params.depth, 10);
  nh_.param<bool>("debug", params.debug, false);
  dvl_range_ = 0;

  // Buffer that will only hold a certain amount of points
  int point_cloud_buffer_Size;
  nh_.param<int>("buffer_size", point_cloud_buffer_Size, 5000);
  point_cloud_buffer_.set_capacity(point_cloud_buffer_Size);

  // TODO: Publish bounds
  service_get_bounds_ = nh_.serviceClient<sub8_msgs::Bounds>("get_bounds");

  // Run the publisher
  timer_ =
      nh_.createTimer(ros::Duration(0.3), std::bind(&OGridGen::publish_big_pointcloud, this, std::placeholders::_1));
  sub_to_imaging_sonar_ = nh_.subscribe("/blueview_driver/ranges", 1, &OGridGen::callback, this);
  sub_to_dvl_ = nh_.subscribe("/dvl/range", 1, &OGridGen::dvl_callback, this);

  mat_ogrid_ = cv::Mat::zeros(int(ogrid_size_ / resolution_), int(ogrid_size_ / resolution_), CV_8U);
  persistant_ogrid_ = cv::Mat(int(ogrid_size_) / resolution_, int(ogrid_size_ / resolution_), CV_32FC1);
  persistant_ogrid_ = 0.5;

  // Make sure alarm integration is ok
  kill_listener_.waitForConnection(ros::Duration(2));
  if (kill_listener_.getNumConnections() < 1)
    throw std::runtime_error("The kill listener isn't connected to the alarm server");
  kill_listener_.start();

  mat_origin_ = cv::Point(0, 0);
}

void OGridGen::dvl_callback(const mil_msgs::RangeStampedConstPtr &dvl)
{
  dvl_range_ = dvl->range;
}
/*
  Looped based on timer_.
  Reads point_cloud_buffer_ and publishes a PointCloud2
  Reads mat_ogrid_ and publish a OccupencyGrid
*/
void OGridGen::publish_big_pointcloud(const ros::TimerEvent &)
{
  // Service call for 'get_bounds'
  sub8_msgs::Bounds get_bound_data;
  if (service_get_bounds_.call(get_bound_data))
  {
    bounds_ = std::vector<cv::Point>();
    bounds_.reserve(4);
    // Populate bounds_ with the data from service call
    for (auto &p : get_bound_data.response.bounds)
    {
      bounds_.push_back(cv::Point(p.x / resolution_ + mat_ogrid_.cols / 2, p.y / resolution_ + mat_ogrid_.rows / 2));
    }

    // Convert bounds_ vector to an array of array and use openCV function to draw a polygon
    const cv::Point *pts = (const cv::Point *)cv::Mat(bounds_).data;
    int npts = cv::Mat(bounds_).rows;
    cv::polylines(mat_ogrid_, &pts, &npts, 1, true, 255, 3, CV_8U, 0);
  }

  // Populate a PCL pointcloud using the point_cloud_buffer_
  pointCloud_->clear();
  pointCloud_->reserve(point_cloud_buffer_.capacity());
  for (auto &p : point_cloud_buffer_)
  {
    pointCloud_->push_back(p);
  }

  // Publish the raw point cloud
  pointCloud_->header.frame_id = "map";
  pcl_conversions::toPCL(ros::Time::now(), pointCloud_->header.stamp);
  pub_point_cloud_raw_.publish(pointCloud_);

  if (params.debug)
  {
    // For debugging a snapshot of current point cloud and filter it and show objects
    pcl::PointCloud<pcl::PointXYZI>::Ptr pointCloud_filtered = classification_.filtered(pointCloud_);
    pub_point_cloud_filtered_.publish(pointCloud_filtered);
    cluster(pointCloud_filtered);
  }
}

/*
  Subscribes to pingmsgs from blueview sonar and saves a plane of pings into a buffer based on sub pose
*/
void OGridGen::callback(const mil_blueview_driver::BlueViewPingPtr &ping_msg)
{
  try  // TODO: Switch to TF2
  {
    listener_.lookupTransform("/map", "/blueview", ros::Time(0), transform_);
  }
  catch (tf::TransformException ex)
  {
    ROS_DEBUG_STREAM("Did not get TF for imaging sonar");
    return;
  }
  if (kill_listener_.isRaised())
    was_killed_ = true;
  else if (was_killed_)
  {
    was_killed_ = false;
    mat_origin_ = cv::Point(transform_.getOrigin().x(), transform_.getOrigin().y());
  }

  pcl::PointCloud<pcl::PointXYZI>::Ptr point_cloud_plane(new pcl::PointCloud<pcl::PointXYZI>());
  for (size_t i = 0; i < ping_msg->ranges.size(); ++i)
  {
    if (ping_msg->intensities.at(i) > min_intensity_)
    {  // TODO: Better thresholding

      // Get x and y of a ping. RIGHT TRIANGLES
      double x_d = ping_msg->ranges.at(i) * cos(ping_msg->bearings.at(i));
      double y_d = ping_msg->ranges.at(i) * sin(ping_msg->bearings.at(i));
      if (std::hypot(x_d, y_d) < params.nearby_threshold)
        continue;

      // Rotate point using TF
      tf::Vector3 vec = tf::Vector3(x_d, y_d, 0);
      tf::Vector3 newVec = transform_.getBasis() * vec;

      // Shift point relative to sub's location
      pcl::PointXYZI point;
      point.x = newVec.x() + transform_.getOrigin().x();
      point.y = newVec.y() + transform_.getOrigin().y();
      point.z = newVec.z() + transform_.getOrigin().z();
      // Ignore points if they are below the ground. This will depend on the current return from the depth.
      if (point.z + dvl_range_ > 0)
        continue;
      // Ignore points if they are below some depth in map frame
      else if (point.z < -params.depth)
        continue;
      point.intensity = ping_msg->intensities.at(i);
      point_cloud_buffer_.push_back(point);
      point_cloud_plane->push_back(point);
    }
  }
  point_cloud_plane->header.frame_id = "map";
  pcl_conversions::toPCL(ros::Time::now(), point_cloud_plane->header.stamp);
  pub_point_cloud_plane_.publish(point_cloud_plane);

  if (params.ogrid)
  {
    process_persistant_ogrid(point_cloud_plane);
    populate_mat_ogrid();
    publish_ogrid();
  }
}
void OGridGen::populate_mat_ogrid()
{
  classification_.zonify(persistant_ogrid_, resolution_, transform_, mat_origin_);
  for (int i = 0; i < persistant_ogrid_.cols; ++i)
  {
    for (int j = 0; j < persistant_ogrid_.rows; ++j)
    {
      float val = persistant_ogrid_.at<float>(cv::Point(i, j));
      if (val > .8)
        mat_ogrid_.at<uchar>(cv::Point(i, j)) = (uchar)WAYPOINT_ERROR_TYPE::OCCUPIED;
      else if (val < .1)
        mat_ogrid_.at<uchar>(cv::Point(i, j)) = (uchar)WAYPOINT_ERROR_TYPE::UNOCCUPIED;
      else
        mat_ogrid_.at<uchar>(cv::Point(i, j)) = (uchar)WAYPOINT_ERROR_TYPE::UNKNOWN;
    }
  }
}

void OGridGen::publish_ogrid()
{
  // Flatten the mat_ogrid_ into a 1D vector for OccupencyGrid message
  nav_msgs::OccupancyGrid rosGrid;
  std::vector<int8_t> data(mat_ogrid_.cols * mat_ogrid_.rows);
  auto out_it = data.begin();
  for (int row = 0; row < mat_ogrid_.rows; ++row)
  {
    auto *p = mat_ogrid_.ptr(row);
    for (int col = 0; col < mat_ogrid_.cols; ++col)
    {
      *out_it = int(*p++);
      out_it++;
    }
  }

  // Publish the ogrid
  rosGrid.header.seq = 0;
  rosGrid.info.resolution = resolution_;
  rosGrid.header.frame_id = "map";
  rosGrid.header.stamp = ros::Time::now();
  rosGrid.info.map_load_time = ros::Time::now();
  rosGrid.info.width = mat_ogrid_.cols;
  rosGrid.info.height = mat_ogrid_.rows;
  rosGrid.info.origin.position.x = mat_origin_.x - ogrid_size_ / 2;
  rosGrid.info.origin.position.y = mat_origin_.y - ogrid_size_ / 2;
  rosGrid.data = data;
  pub_grid_.publish(rosGrid);
}

void OGridGen::process_persistant_ogrid(pcl::PointCloud<pcl::PointXYZI>::Ptr point_cloud_plane)
{
  nh_.param<float>("hit_prob", hit_prob_, 0.1);

  for (auto &point_pcl : point_cloud_plane->points)
  {
    // Check if point is inside the potential ogrid
    cv::Rect rect(cv::Point(0, 0), persistant_ogrid_.size());
    cv::Point p(point_pcl.x / resolution_ + persistant_ogrid_.cols / 2 - mat_origin_.x / resolution_,
                point_pcl.y / resolution_ + persistant_ogrid_.rows / 2 - mat_origin_.y / resolution_);
    if (rect.contains(p))
    {
      if (persistant_ogrid_.at<float>(p.y, p.x) < 1)
      {
        persistant_ogrid_.at<float>(p.y, p.x) += hit_prob_;
      }
    }
  }
}

mil_msgs::PerceptionObjectArray OGridGen::cluster(pcl::PointCloud<pcl::PointXYZI>::Ptr pc)
{
  int id = 0;
  visualization_msgs::MarkerArray markers;
  mil_msgs::PerceptionObjectArray objects;
  // Cluster points into objects
  std::vector<pcl::PointIndices> cluster_indices = classification_.clustering(pc);

  // Iterate objects
  for (auto it = cluster_indices.begin(); it != cluster_indices.end(); ++it)
  {
    mil_msgs::PerceptionObject object;
    pcl::CentroidPoint<pcl::PointXYZI> centroid;
    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_cluster(new pcl::PointCloud<pcl::PointXYZI>);
    for (std::vector<int>::const_iterator pit = it->indices.begin(); pit != it->indices.end(); ++pit)
    {
      auto p = pc->points[*pit];
      cloud_cluster->push_back(p);
      centroid.add(pc->points[*pit]);

      // Add point to object's array
      geometry_msgs::Point32 geo_p;
      geo_p.x = p.x;
      geo_p.y = p.y;
      geo_p.z = p.z;
      object.points.emplace_back(geo_p);
    }
    cloud_cluster->width = static_cast<uint32_t>(cloud_cluster->points.size());
    cloud_cluster->height = 1;
    cloud_cluster->is_dense = true;
    pcl::PointXYZI minPt, maxPt;
    pcl::getMinMax3D(*cloud_cluster, minPt, maxPt);

    pcl::PointXYZI c;
    centroid.get(c);

    // Init object header
    object.header.stamp = ros::Time::now();
    object.header.frame_id = "map";
    object.classification = "object";
    object.labeled_classification = "unknown";
    object.pose.position.x = c.x;
    object.pose.position.y = c.y;
    object.pose.position.z = c.z;
    object.pose.orientation.w = 1;
    object.scale.x = maxPt.x - minPt.x;
    object.scale.y = maxPt.y - minPt.y;
    object.scale.z = maxPt.z - minPt.z;
    objects.objects.push_back(object);

    visualization_msgs::Marker marker;
    marker.header.frame_id = "map";
    marker.id = id++;
    marker.type = visualization_msgs::Marker::SPHERE;
    marker.pose.position.x = c.x;
    marker.pose.position.y = c.y;
    marker.pose.position.z = c.z;
    marker.scale.x = maxPt.x - minPt.x;
    marker.scale.y = maxPt.y - minPt.y;
    marker.scale.z = maxPt.z - minPt.z;
    marker.color.a = 1.0;
    marker.color.b = 1.0;
    markers.markers.push_back(marker);
  }

  pub_markers_.publish(markers);
  return objects;
}

bool OGridGen::clear_ogrid_callback(std_srvs::Trigger::Request &req, std_srvs::Trigger::Response &res)
{
  persistant_ogrid_ = 0.5;
  res.success = true;
  return true;
}

bool OGridGen::clear_pcl_callback(std_srvs::Trigger::Request &req, std_srvs::Trigger::Response &res)
{
  point_cloud_buffer_.clear();
  res.success = true;
  return true;
}

bool OGridGen::get_objects_callback(mil_msgs::ObjectDBQuery::Request &req, mil_msgs::ObjectDBQuery::Response &res)
{
  pcl::PointCloud<pcl::PointXYZI>::Ptr pointCloud_filtered = classification_.filtered(pointCloud_);
  if (pointCloud_filtered->size() < 1)
  {
    res.found = false;
    return false;
  }
  mil_msgs::PerceptionObjectArray objects = cluster(pointCloud_filtered);
  res.found = true;
  res.objects = objects.objects;
  return true;
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "ogrid_pointcloud");
  OGridGen oGridGen;
  ros::spin();
}