#pragma once

#include <navigator_vision_lib/colorizer/common.hpp>  // Common includes are here
#include <navigator_vision_lib/colorizer/color_observation.hpp>
#include <tf/transform_listener.h>
#include <image_geometry/pinhole_camera_model.h>
#include <opencv2/core/eigen.hpp>

namespace nav{
  
class CameraObserver
{
  using CamStream = ROSCameraStream<cv::Vec3b>;
  
  ros::NodeHandle _nh;

  ROSCameraStream<cv::Vec3b> _cam_stream;
  tf::TransformListener _tf_listener;

  std::string _err_msg{""};
  bool _ok{false};
  
public:
  CameraObserver(ros::NodeHandle &nh, std::string &pcd_in_topic, std::string &cam_topic, size_t hist_size);

  std::shared_ptr<image_geometry::PinholeCameraModel> getCameraModelPtr() const
  {
    return _cam_stream.getCameraModelPtr();
  }

  std::vector<ColorObservation> operator()(const PCD<pcl::PointXYZ>::ConstPtr &pcd)
  {
    return std::vector<ColorObservation>{};
  }

  bool ok() const
  {
    return _ok && ros::ok();
  }

 ColorObservation::VecImg get_color_observations(const PCD<pcl::PointXYZ>::ConstPtr &pcd);

};

}  // namespace nav
