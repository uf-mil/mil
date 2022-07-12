#include <tf2/LinearMath/Quaternion.h>

#include <opencv2/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <point_cloud_object_detection_and_recognition/object.hpp>

namespace pcodar
{
Object::Object(point_cloud_ptr const& _pc, uint id, KdTreePtr const& search_tree)
{
  set_id(id);
  set_classification("UNKNOWN");
  update_points(_pc, search_tree);
}

void Object::update_points(point_cloud_ptr const& pc, KdTree::Ptr const& search_tree)
{
  points_ = pc;
  search_tree_ = search_tree;
  update_msg();
}

KdTree::Ptr Object::get_search_tree() const
{
  return search_tree_;
}

point_cloud const& Object::get_points() const
{
  return *points_;
}

point_cloud_ptr Object::get_points_ptr() const
{
  return points_;
}

mil_msgs::PerceptionObject const& Object::as_msg() const
{
  return msg_;
}

point_t const& Object::get_center() const
{
  return center_;
}

void Object::set_classification(std::string const& classification)
{
  msg_.labeled_classification = classification;
}

void Object::set_id(uint id)
{
  msg_.id = id;
}

void Object::update_msg()
{
  // Classification field is unused, always set to empty string
  msg_.classification = "";
  msg_.header.frame_id = "enu";
  msg_.header.stamp = ros::Time();
  msg_.points.clear();

  if (points_->empty())
    return;
  std::vector<cv::Point2f> cv_points;
  double min_z = std::numeric_limits<double>::max();
  double max_z = -std::numeric_limits<double>::max();
  for (point_t const& point : *points_)
  {
    cv_points.emplace_back(point.x, point.y);
    if (point.z > max_z)
    {
      max_z = point.z;
    }

    if (point.z < min_z)
    {
      min_z = point.z;
    }
    geometry_msgs::Point32 g_point;
    g_point.x = point.x;
    g_point.y = point.y;
    g_point.z = point.z;

    msg_.points.emplace_back(g_point);
  }
  cv::RotatedRect rect = cv::minAreaRect(cv_points);
  center_.x = rect.center.x;
  center_.y = rect.center.y;
  center_.z = (max_z + min_z) / 2.0;
  msg_.pose.position.x = rect.center.x;
  msg_.pose.position.y = rect.center.y;
  msg_.pose.position.z = center_.z;
  msg_.scale.x = rect.size.width;
  msg_.scale.y = rect.size.height;
  msg_.scale.z = max_z - min_z;

  tf2::Quaternion quat;
  quat.setRPY(0., 0., rect.angle * 3.14159 / 180);
  quat.normalize();
  msg_.pose.orientation.x = quat.x();
  msg_.pose.orientation.y = quat.y();
  msg_.pose.orientation.z = quat.z();
  msg_.pose.orientation.w = quat.w();
}

}  // namespace pcodar
