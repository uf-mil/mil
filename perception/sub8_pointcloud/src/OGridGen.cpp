#include "OGridGen.hpp"

//TODO: Add service call to clear ogrid

OGridGen::OGridGen() : nh_(ros::this_node::getName()), classification_(&nh_)
{
  //The publishers
  pub_grid_ = nh_.advertise<nav_msgs::OccupancyGrid>("ogrid", 10, true);
  pub_point_cloud_ = nh_.advertise<pcl::PointCloud<pcl::PointXYZI>>("point_cloud", 1);

  //Resolution is meters/pixel
  nh_.param<float>("resolution_", resolution_, 0.2f);
  nh_.param<float>("ogrid_size_", ogrid_size_, 50.f);
  //Ignore points that are below the potential pool
  nh_.param<float>("pool_depth_", pool_depth_, 7.f);
  
  //Buffer that will only hold a certain amount of points
  int point_cloud_buffer_Size;
  nh_.param<int>("buffer_size", point_cloud_buffer_Size, 5000);
  point_cloud_buffer_.set_capacity(point_cloud_buffer_Size);

  //TODO: Publish bounds
  service_get_bounds_ = nh_.serviceClient<sub8_msgs::Bounds>("get_bounds");

  //Run the publisher
  timer_ = nh_.createTimer(ros::Duration(0.3), std::bind(&OGridGen::publish_ogrid, this, std::placeholders::_1));
  sub_to_imaging_sonar_ = nh_.subscribe("/blueview_driver/ranges", 1, &OGridGen::callback, this);

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

  // Flatten the mat_ogrid_ into a 1D vector for OccupencyGrid message
  nav_msgs::OccupancyGrid rosGrid;
  std::vector<int8_t> data(mat_ogrid_.cols*mat_ogrid_.rows);
  auto out_it = data.begin();
  for(int row = 0; row < mat_ogrid_.rows; ++row) {
      auto* p = mat_ogrid_.ptr(row);
      for(int col = 0; col < mat_ogrid_.cols; ++col) {
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
  rosGrid.info.origin.position.x = -ogrid_size_ / 2;
  rosGrid.info.origin.position.y = -ogrid_size_ / 2;
  rosGrid.data = data;
  pub_grid_.publish(rosGrid);

  // Populate a PCL pointcloud using the point_cloud_buffer_
  pcl::PointCloud<pcl::PointXYZI>::Ptr pointCloud(new pcl::PointCloud<pcl::PointXYZI>());
  pointCloud->reserve(point_cloud_buffer_.capacity());
  for (auto &p : point_cloud_buffer_)
  {
    pointCloud->push_back(p);
  }
  
  // Publish the point cloud
  pointCloud->header.frame_id = "map";
  pcl_conversions::toPCL(ros::Time::now(), pointCloud->header.stamp);
  pub_point_cloud_.publish(pointCloud);

  // Do some algorithms using the PCL pointcloud
  // classification_.segment(pointCloud);
  // classification_.findNormals(pointCloud);
  classification_.filtered(pointCloud);

}


/*

  Subscribes to pingmsgs from blueview sonar and saves a plane of pings into a buffer based on sub pose

*/
void OGridGen::callback(const mil_blueview_driver::BlueViewPingPtr &ping_msg)
{
  try //TODO: Switch to TF2
  {
    listener_.lookupTransform("/map", "/blueview", ros::Time(0), transform_);
  }
  catch (tf::TransformException ex)
  {
    ROS_DEBUG_STREAM("Did not get TF for imaging sonar");
    return;
  }

  //Clear the last ogrid
  mat_ogrid_ = cv::Mat::zeros(int(ogrid_size_ / resolution_), int(ogrid_size_ / resolution_), CV_8U);
  for (size_t i = 0; i < ping_msg->ranges.size(); ++i)
  {
    if (ping_msg->intensities.at(i) > 2000)
    {  // TODO: Better thresholding

      // Get distance. RIGHT TRIANGLES
      double x_d = ping_msg->ranges.at(i) * cos(ping_msg->bearings.at(i));
      double y_d = ping_msg->ranges.at(i) * sin(ping_msg->bearings.at(i));

      // Transform point using TF
      tf::Vector3 vec = tf::Vector3(x_d, y_d, 0);
      tf::Vector3 newVec = transform_.getBasis() * vec;

      // Where to draw in opencv mat
      int x = newVec.x() / resolution_ + transform_.getOrigin().x() / resolution_ + mat_ogrid_.cols / 2;
      int y = newVec.y() / resolution_ + transform_.getOrigin().y() / resolution_ + mat_ogrid_.rows / 2;

      // Check if point is inside the OGRID
      cv::Rect rect(cv::Point(), mat_ogrid_.size());
      cv::Point p(x, y);
      if (rect.contains(p) && mat_ogrid_.at<uchar>(y, x) == 0 && newVec.z() > -pool_depth_)
      {
        mat_ogrid_.at<uchar>(y, x) = 99;
        pcl::PointXYZI point;
        point.x = newVec.x() + transform_.getOrigin().x();
        point.y = newVec.y() + transform_.getOrigin().y();
        point.z = newVec.z() + transform_.getOrigin().z();
        point.intensity = ping_msg->intensities.at(i);
        point_cloud_buffer_.push_back(point);
      }
    }
  }
  // cv::transpose(mat_ogrid_, mat_ogrid_);
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "ogridgen");
  OGridGen oGridGen;
  ros::spin();
}