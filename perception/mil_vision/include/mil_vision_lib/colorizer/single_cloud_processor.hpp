#pragma once

#include <mil_vision_lib/image_acquisition/ros_camera_stream.hpp>
#include <mil_vision_lib/colorizer/camera_observer.hpp>
#include <mil_vision_lib/colorizer/common.hpp>
#include <mil_tools/mil_tools.hpp>
#include <pcl_ros/transforms.h>
#include <tf/transform_listener.h>

namespace nav {

class SingleCloudProcessor
{
public:
  SingleCloudProcessor(ros::NodeHandle nh, std::string &in_pcd_topic, size_t hist_size);
  void operator()(const PCD<pcl::PointXYZ>::ConstPtr &pcd);
  bool ok() const { return _ok && ros::ok(); }

private:
  ros::NodeHandle _nh;
  std::string in_pcd_topic;
  size_t _hist_size;  // image history buffer size
  
  // CameraObserver creates point color observations for a specific camera
  UPtrVector<CameraObserver> _camera_observers;

  // Inter-thread communication
  std::promise<void> _start_work_prom;
  std::shared_future<void> _start_work_fut{_start_work_prom.get_future()};
  std::vector<std::promise<void>> _worker_done_proms;
  std::vector<std::future<void>> _worker_done_futs;


  int seq = 0;

  bool _ok{false};
  std::string _err_msg{""};

};

}  // namespace nav
