#pragma once

#include "pcodar_types.hpp"

#include <sensor_msgs/PointCloud2.h>

#include <boost/circular_buffer.hpp>

#include <pcl/filters/extract_indices.h>

namespace pcodar
{
/**
 * Buffer of n most recently added pointclouds, used to assemble a persistent pointcloud of pointclouds accumulated over
 * time.
 */
class PointCloudCircularBuffer
{
public:
  PointCloudCircularBuffer();
  /// Get accumulated pointcloud from buffered pointclouds, or nullptr if N pointclouds have not yet been added
  point_cloud_ptr get_point_cloud();
  /// Add a pointcloud to the back of the buffer, deleting the oldest if it is full.
  void add_point_cloud(const point_cloud_ptr& pc);
  /// Update the number of pointclouds from the dynamic reconfigure object
  void update_config(Config const& config);

private:
  /// Accumulated pointcloud kept up to date with each call to add_point_cloud
  point_cloud_ptr mega_cloud_;
  /// Internal buffer of pointclouds
  /// @TODO consider more efficent implementation, like storing indicies of older pointclouds so
  ///       the accumulated cloud is not reassembled on each addition
  boost::circular_buffer<point_cloud_ptr> prev_clouds_;
};
}  // pcodar namespace
