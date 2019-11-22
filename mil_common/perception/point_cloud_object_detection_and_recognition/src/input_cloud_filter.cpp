#include <point_cloud_object_detection_and_recognition/input_cloud_filter.hpp>

namespace pcodar
{
InputCloudFilter::InputCloudFilter()
{
  bounds_filter_.setDim(2);
  bounds_filter_.setCropOutside(true);

  robot_filter_.setNegative(true);
}

void InputCloudFilter::filter(point_cloud_const_ptr in, point_cloud& pc)
{
  // Filter out bounds
  point_cloud_ptr tmp(boost::make_shared<point_cloud>());
  bounds_filter_.setInputCloud(in);
  bounds_filter_.filter(*tmp);

  robot_filter_.setInputCloud(tmp);
  robot_filter_.filter(pc);
}

void InputCloudFilter::set_bounds(point_cloud_ptr bounds)
{
  pcl::Vertices indicies;
  indicies.vertices.reserve(bounds->size());
  for (size_t i = 0; i < bounds->size(); ++i)
    indicies.vertices.push_back(i);
  std::vector<pcl::Vertices> hull = { indicies };

  bounds_filter_.setHullCloud(bounds);
  bounds_filter_.setHullIndices(hull);
}

void InputCloudFilter::set_robot_footprint(Eigen::Vector4f const& min, Eigen::Vector4f const& max)
{
  robot_filter_.setMin(min);
  robot_filter_.setMax(max);
}

void InputCloudFilter::set_robot_pose(Eigen::Affine3d const& transform)
{
  Eigen::Affine3f transform_float = transform.cast<float>();
  robot_filter_.setTransform(transform_float);
}

}  // namespace pcodar
