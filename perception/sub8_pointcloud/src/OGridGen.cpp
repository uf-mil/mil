#include "OGridGen.hpp"

// TODO: Add service call to clear ogrid

OGridGen::OGridGen()
  : nh_(ros::this_node::getName()), kill_listener_(nh_, "kill"), was_killed_(true), classification_(&nh_)
{
  // The publishers
  pub_grid_ = nh_.advertise<nav_msgs::OccupancyGrid>("ogrid", 10, true);
  pub_point_cloud_filtered_ = nh_.advertise<pcl::PointCloud<pcl::PointXYZI>>("point_cloud/filtered", 1);
  pub_point_cloud_raw_ = nh_.advertise<pcl::PointCloud<pcl::PointXYZI>>("point_cloud/raw", 1);
  pub_point_cloud_plane_ = nh_.advertise<pcl::PointCloud<pcl::PointXYZI>>("point_cloud/plane", 1);

  // Resolution is meters/pixel
  nh_.param<float>("resolution", resolution_, 0.2f);
  nh_.param<float>("ogrid_size", ogrid_size_, 91.44);
  // Ignore points that are below the potential pool
  nh_.param<int>("min_intensity", min_intensity_, 2000);
  dvl_range_ = 0;

  // Buffer that will only hold a certain amount of points
  int point_cloud_buffer_Size;
  nh_.param<int>("buffer_size", point_cloud_buffer_Size, 5000);
  point_cloud_buffer_.set_capacity(point_cloud_buffer_Size);

  // TODO: Publish bounds
  service_get_bounds_ = nh_.serviceClient<sub8_msgs::Bounds>("get_bounds");

  // Run the publisher
  timer_ = nh_.createTimer(ros::Duration(0.3), std::bind(&OGridGen::publish_ogrid, this, std::placeholders::_1));
  sub_to_imaging_sonar_ = nh_.subscribe("/blueview_driver/ranges", 1, &OGridGen::callback, this);
  sub_to_dvl_ = nh_.subscribe("/dvl/ranges", 1, &OGridGen::dvl_callback, this);

  mat_ogrid_ = cv::Mat::zeros(int(ogrid_size_ / resolution_), int(ogrid_size_ / resolution_), CV_8U);

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
void OGridGen::publish_ogrid(const ros::TimerEvent &)
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
  pcl::PointCloud<pcl::PointXYZI>::Ptr pointCloud(new pcl::PointCloud<pcl::PointXYZI>());
  pointCloud->reserve(point_cloud_buffer_.capacity());
  for (auto &p : point_cloud_buffer_)
  {
    pointCloud->push_back(p);
  }

  // Publish the raw point cloud
  pointCloud->header.frame_id = "map";
  pcl_conversions::toPCL(ros::Time::now(), pointCloud->header.stamp);
  pub_point_cloud_raw_.publish(pointCloud);

  // Publish the filtered cloud
  pcl::PointCloud<pcl::PointXYZI>::Ptr pointCloud_filtered = classification_.filtered(pointCloud);
  pub_point_cloud_filtered_.publish(pointCloud_filtered);

  // TODO: Implement some k-nearest neighbors algorithm to filter ogrid

  mat_ogrid_ = cv::Scalar((uchar)WAYPOINT_ERROR_TYPE::UNKNOWN);
  // Populate the Ogrid by projecting down the pointcloud
  for (auto &point_pcl : pointCloud_filtered->points)
  {
    // Check if point is inside the potential ogrid
    cv::Rect rect(cv::Point(0, 0), mat_ogrid_.size());
    cv::Point p(point_pcl.x / resolution_ + mat_ogrid_.cols / 2 - mat_origin_.x / resolution_,
                point_pcl.y / resolution_ + mat_ogrid_.rows / 2 - mat_origin_.y / resolution_);
    if (rect.contains(p))
    {
      mat_ogrid_.at<uchar>(p.y, p.x) = (uchar)WAYPOINT_ERROR_TYPE::OCCUPIED;
    }
  }

  classification_.zonify(mat_ogrid_, resolution_, transform_, mat_origin_);

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

      // Rotate point using TF
      tf::Vector3 vec = tf::Vector3(x_d, y_d, 0);
      tf::Vector3 newVec = transform_.getBasis() * vec;

      // Shift point relative to sub's location
      pcl::PointXYZI point;
      point.x = newVec.x() + transform_.getOrigin().x();
      point.y = newVec.y() + transform_.getOrigin().y();
      point.z = newVec.z() + transform_.getOrigin().z();
      if (point.z + dvl_range_ > 0)
        continue;
      point.intensity = ping_msg->intensities.at(i);
      point_cloud_buffer_.push_back(point);
      point_cloud_plane->push_back(point);
    }
  }
  point_cloud_plane->header.frame_id = "map";
  pcl_conversions::toPCL(ros::Time::now(), point_cloud_plane->header.stamp);
  pub_point_cloud_plane_.publish(point_cloud_plane);
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "ogrid_pointcloud");
  OGridGen oGridGen;
  ros::spin();
}