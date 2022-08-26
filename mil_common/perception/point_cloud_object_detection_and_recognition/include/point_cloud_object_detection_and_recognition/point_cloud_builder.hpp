#pragma once

#include <pcl/filters/extract_indices.h>
#include <sensor_msgs/PointCloud2.h>

#include <boost/circular_buffer.hpp>
#include <list>
#include <thread>
#include <unordered_map>
#include <vector>

#include "pcodar_types.hpp"

template <class T>

inline void hash_combine(std::size_t& seed, const T& v)
{
  std::hash<T> hasher;
  seed ^= hasher(v) + 0x9e3779b9 + (seed << 6) + (seed >> 2);
}

struct point_overload : pcodar::point_t
{
};

inline bool operator==(const point_overload& lhs, const point_overload& rhs)
{
  return lhs.x == rhs.x && lhs.y == rhs.y && lhs.z == rhs.z;
}

template <>
struct std::hash<point_overload>
{
  size_t operator()(const point_overload& v) const noexcept
  {
    size_t h = std::hash<float>()(v.x);
    hash_combine(h, v.y);
    hash_combine(h, v.z);
    return h;
  }
};

namespace pcodar
{
/**
 * Buffer of n most recently added pointclouds, used to assemble a persistent pointcloud of pointclouds accumulated
 * over time.
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
  /// Clear buffer
  void clear();

private:
  void handle_thread(const std::vector<std::reference_wrapper<const point_t>>& points_list_,
                     const std::vector<int>& cumulative_sizes_, std::vector<size_t>& indices_, int index);
  void handle_old_points_deletion(const std::vector<std::reference_wrapper<const point_t>>& points_list_,
                                  std::chrono::time_point<std::chrono::high_resolution_clock>& start);
  void reset_atomics();

  /// Accumulated pointcloud kept up to date with each call to add_point_cloud
  point_cloud_ptr mega_cloud_;
  /// Internal buffer of pointclouds
  /// @TODO consider more efficient implementation, like storing indices of older pointclouds so
  ///       the accumulated cloud is not reassembled on each addition
  boost::circular_buffer<point_cloud_ptr> prev_clouds_;

  std::unordered_map<point_overload, std::vector<size_t>> points_to_cloud_;
  std::unordered_map<size_t, std::vector<point_t*>> points_list_;

  static unsigned int number_threads_;
  std::vector<std::thread> threads_;
  std::atomic<bool>* completed_threads_;
  size_t counter_;
  size_t capacity_;
};
}  // namespace pcodar
