#include <point_cloud_object_detection_and_recognition/ogrid_manager.hpp>

namespace pcodar
{

ogrid_manager::ogrid_manager() /*
  height_meters_(params.ogrid_height_meters),
  width_meters_(params.ogrid_width_meters),
  resolution_meters_per_cell_(params.ogrid_resolution_meters_per_cell),
  ogrid_inflation_cells_(1.0) //resolution_meters_per_cell_ / params.ogrid_inflation_meters)
*/
{
}

void ogrid_manager::initialize(ros::NodeHandle& nh) {
  pub_ogrid_ = nh.advertise<nav_msgs::OccupancyGrid>("/ogrid", 5);
}

void ogrid_manager::draw_boundary() {
/*
  std::vector<cv::Point>bounds(pcodar::boundary.size());
  for(int i = 0; i < bounds.size(); ++i) {
    bounds[i] = point_in_ogrid(bounds[i]);
  }

  for(int i = 0; i < bounds.size(); ++i) {
    // std::cout << bounds[i] << std::endl;
    cv::circle(ogrid_mat_, bounds[i], 15, cv::Scalar(99), -1);
  }
  const cv::Point *pts = (const cv::Point*) cv::Mat(bounds).data;
  int npts = cv::Mat(bounds).rows;

  cv::polylines(ogrid_mat_, &pts, &npts, 1, true, cv::Scalar(99), 5);
*/
}

cv::Point ogrid_manager::point_in_ogrid(point_t point)
{
  return cv::Point(point.x / resolution_meters_per_cell_ + ogrid_.info.width/2,
                   point.y / resolution_meters_per_cell_ + + ogrid_.info.height/2);
}

void ogrid_manager::update_ogrid(ObjectMap const& objects)
{
  // Clear ogrid
  ogrid_mat_ = cv::Scalar(0);

  // Draw border on ogrid
  draw_boundary();

  for (auto const& pair : objects.objects_)
  {
    Object const& object = pair.second;
    for(const auto &point : object.points_) {
      cv::Point center(point_in_ogrid(point));
      cv::circle(ogrid_mat_, center, inflation_cells_, cv::Scalar(99), -1);
    }
  }

  ogrid_.header.stamp = ros::Time::now();

  pub_ogrid_.publish(ogrid_);
}

void ogrid_manager::update_config(Config const& config)
{
  ROS_INFO("WIDTH=%f HEIGHT=%f resolution=%f, inflation=%f", config.ogrid_width_meters, config.ogrid_height_meters, config.ogrid_resolution_meters_per_cell, config.ogrid_inflation_meters);

  width_meters_ = config.ogrid_width_meters;
  height_meters_ = config.ogrid_height_meters;
  resolution_meters_per_cell_ = config.ogrid_resolution_meters_per_cell;
  inflation_cells_ = config.ogrid_inflation_meters / resolution_meters_per_cell_;

  ogrid_.info.resolution = resolution_meters_per_cell_;
  ogrid_.info.width = resolution_meters_per_cell_ * width_meters_;
  ogrid_.info.height = resolution_meters_per_cell_ * height_meters_;
  ogrid_.info.origin.position.x = -10000*0.09/2;
  ogrid_.info.origin.position.y = -10000*0.09/2;
  ogrid_.info.origin.orientation.w = 1;
  ogrid_.data.resize(ogrid_.info.width * ogrid_.info.height);
  ogrid_mat_ = cv::Mat(cv::Size(ogrid_.info.width, ogrid_.info.height), CV_8UC1, ogrid_.data.data());
}

} // namespace pcodar
