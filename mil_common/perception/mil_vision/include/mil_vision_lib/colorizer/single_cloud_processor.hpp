#pragma once

#include <pcl_ros/transforms.h>
#include <tf/transform_listener.h>
#include <mil_tools/mil_tools.hpp>
#include <mil_vision_lib/colorizer/camera_observer.hpp>
#include <mil_vision_lib/colorizer/common.hpp>
#include <mil_vision_lib/image_acquisition/ros_camera_stream.hpp>

namespace mil_vision
{
class SingleCloudProcessor
{
public:
  /**
   * Constructs a new cloud processor.
   *
   * @param nh The node handle for the class.
   * @param in_pcd_topic The name of the topic supplying point clouds.
   * @param hist_size The size of the image history buffer.
   */
  SingleCloudProcessor(ros::NodeHandle nh, std::string &in_pcd_topic, size_t hist_size);

  /**
   * Incomplete method. Aside from priting debug statements, does no actions.
   */
  void operator()(const PCD<pcl::PointXYZ>::ConstPtr &pcd);

  /**
   * Returns true if ROS and the class are functioning properly.
   */
  bool ok() const
  {
    return _ok && ros::ok();
  }

private:
  ros::NodeHandle _nh;
  std::string in_pcd_topic;
  size_t _hist_size;  // image history buffer size

  // CameraObserver creates point color observations for a specific camera
  UPtrVector<CameraObserver> _camera_observers;

  // Inter-thread communication
  std::promise<void> _start_work_prom;
  std::shared_future<void> _start_work_fut{ _start_work_prom.get_future() };
  std::vector<std::promise<void>> _worker_done_proms;
  std::vector<std::future<void>> _worker_done_futs;

  int seq = 0;

  bool _ok{ false };
  std::string _err_msg{ "" };
};

}  // namespace mil_vision
