#include <tf2/LinearMath/Quaternion.h>
#include <tf2/utils.h>

#include <point_cloud_object_detection_and_recognition/ogrid_manager.hpp>

namespace pcodar
{
OgridManager::OgridManager()
{
}

void OgridManager::initialize(ros::NodeHandle& nh)
{
  pub_ogrid_ = nh.advertise<nav_msgs::OccupancyGrid>("/ogrid", 5);
}

void OgridManager::draw_boundary()
{
  std::vector<cv::Point> bounds(bounds_->size());
  for (int i = 0; i < bounds.size(); ++i)
  {
    bounds[i] = point_in_ogrid(bounds_->points[i]);
  }

  for (int i = 0; i < bounds.size(); ++i)
  {
    // std::cout << bounds[i] << std::endl;
    cv::circle(ogrid_mat_, bounds[i], 15, cv::Scalar(99), -1);
  }
  const cv::Point* pts = (const cv::Point*)cv::Mat(bounds).data;
  int npts = cv::Mat(bounds).rows;

  cv::polylines(ogrid_mat_, &pts, &npts, 1, true, cv::Scalar(99), 5);
}

void OgridManager::set_bounds(point_cloud_ptr pc)
{
  bounds_ = pc;
}

cv::Point OgridManager::point_in_ogrid(point_t point)
{
  double x = (point.x - ogrid_.info.origin.position.x) / resolution_meters_per_cell_;
  double y = (point.y - ogrid_.info.origin.position.y) / resolution_meters_per_cell_;
  return cv::Point(x, y);
}

void OgridManager::update_ogrid(ObjectMap const& objects)
{
  // Clear ogrid
  ogrid_mat_ = cv::Scalar(0);

  // Draw border on ogrid
  draw_boundary();

  for (auto const& pair : objects.objects_)
  {
    Object const& object = pair.second;
    mil_msgs::PerceptionObject const& msg = object.as_msg();

    // In simulation, use bounding box
    if (object.get_points().empty())
    {
      tf2::Quaternion quat(msg.pose.orientation.x, msg.pose.orientation.y, msg.pose.orientation.z,
                           msg.pose.orientation.w);
      double pitch, roll, yaw;
      tf2::getEulerYPR(quat, yaw, pitch, roll);
      cv::RotatedRect rect(cv::Point2f(msg.pose.position.x, msg.pose.position.y), cv::Size2f(msg.scale.x, msg.scale.y),
                           yaw);
      cv::Point2f vertices[4];
      cv::Point vertices_fixed[4];
      rect.points(vertices);
      for (size_t i = 0; i < 4; ++i)
        vertices_fixed[i] = point_in_ogrid(point_t(vertices[i].x, vertices[i].y, 0));
      cv::fillConvexPoly(ogrid_mat_, vertices_fixed, 4, cv::Scalar(99));
    }

    // Otherwise draw individual points
    else
    {
      for (const auto& point : object.get_points())
      {
        cv::Point center(point_in_ogrid(point));
        cv::circle(ogrid_mat_, center, inflation_cells_, cv::Scalar(99), -1);
      }
    }
  }

  ogrid_.header.stamp = ros::Time::now();

  pub_ogrid_.publish(ogrid_);
}

void OgridManager::update_config(Config const& config)
{
  width_meters_ = config.ogrid_width_meters;
  height_meters_ = config.ogrid_height_meters;
  resolution_meters_per_cell_ = config.ogrid_resolution_meters_per_cell;
  inflation_cells_ = config.ogrid_inflation_meters / resolution_meters_per_cell_;

  ogrid_.header.frame_id = "enu";
  ogrid_.info.resolution = resolution_meters_per_cell_;
  ogrid_.info.width = width_meters_ / resolution_meters_per_cell_;
  ogrid_.info.height = height_meters_ / resolution_meters_per_cell_;
  ogrid_.info.origin.position.x = -1. * width_meters_ / 2.;  //-1. * width_meters_ * resolution_meters_per_cell_ / 2.;
  ogrid_.info.origin.position.y =
      -1. * height_meters_ / 2.;                             // -1. * height_meters_ * resolution_meters_per_cell_ / 2.;
  ogrid_.info.origin.orientation.w = 1;
  ogrid_.data.resize(ogrid_.info.width * ogrid_.info.height);
  ogrid_mat_ = cv::Mat(cv::Size(ogrid_.info.width, ogrid_.info.height), CV_8UC1, ogrid_.data.data());
}

}  // namespace pcodar
