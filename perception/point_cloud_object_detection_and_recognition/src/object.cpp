#include <point_cloud_object_detection_and_recognition/object.hpp>

#include <opencv2/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>

#include <tf2/LinearMath/Quaternion.h>

namespace pcodar
{
Object::Object(point_cloud const& _pc)
{
  update_points(_pc);
}

void Object::update_points(point_cloud const& pc)
{
  points_ = pc;
  update_msg();
}

void Object::update_msg()
{
  msg_.classification = "UNKNOWN";
  msg_.header.frame_id = "enu";
  msg_.header.stamp = ros::Time();

  if (points_.empty()) return;
  std::vector<cv::Point2f> cv_points;
  double min_z = std::numeric_limits<double>::max();
  double max_z = -std::numeric_limits<double>::max();
  for (point_t const& point : points_)
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
