#include "waypoint_validity.hpp"

bool WaypointValidity::check_if_hit(cv::Point center, cv::Size sub_size)
{
  for (int x = center.x - sub_size.width / 2; x < center.x + sub_size.width / 2; ++x)
  {
    for (int y = center.y - sub_size.height / 2; y < center.y + sub_size.height / 2; ++y)
    {
      if (ogrid_map_->data.at(x + y * ogrid_map_->info.width) == 99)
      {
        return true;
      }
    }
  }
  return false;
}

void WaypointValidity::ogrid_callback(const nav_msgs::OccupancyGridConstPtr &ogrid_map)
{
  this->ogrid_map_ = ogrid_map;
}

void WaypointValidity::is_waypoint_valid(const geometry_msgs::Pose &waypoint, bool &valid, int &error)
{
  if (waypoint.position.z > 0)
  {
    valid = false;
    error = 5;
    return;
  }
  cv::Point where_sub = cv::Point(waypoint.position.x / ogrid_map_->info.resolution + ogrid_map_->info.width / 2,
                                  waypoint.position.y / ogrid_map_->info.resolution + ogrid_map_->info.height / 2);

  if (ogrid_map_->data.at(where_sub.x + where_sub.y * ogrid_map_->info.width) == 50)
  {
    valid = false;
    error = 50;
  }

  if (check_if_hit(where_sub, cv::Size(5, 5)))
  {
    valid = false;
    error = 99;
  }
}

WaypointValidity::WaypointValidity(ros::NodeHandle &nh)
{
  nh_ = &nh;
  sub_ = nh_->subscribe<nav_msgs::OccupancyGrid>("/ogridgen/ogrid", 1,
                                                 boost::bind(&WaypointValidity::ogrid_callback, this, _1));
}