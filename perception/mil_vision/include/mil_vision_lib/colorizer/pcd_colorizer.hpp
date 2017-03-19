#pragma once

#include <mil_vision_lib/colorizer/common.hpp>
#include <mil_vision_lib/pcd_sub_pub_algorithm.hpp>
#include <mil_vision_lib/image_acquisition/ros_camera_stream.hpp>
#include <mil_vision_lib/colorizer/single_cloud_processor.hpp>
#include <mil_tools/mil_tools.hpp>
#include <pcl_ros/transforms.h>
#include <tf/transform_listener.h>

namespace nav{

class PcdColorizer{
  /*
    This class takes adds color information to XYZ only point clouds if the
    points in the cloud can be observed by a camera that takes color images
    Note: needs accurate TF because it uses the ros tf package to transform 
    arbitrary point cloud frames into the frame of the color pinhole camera
    Note: the rgb camera topic should be streaming rectified images
  */

  using CamStream = ROSCameraStream<cv::Vec3b>;

public:

  PcdColorizer(ros::NodeHandle nh, std::string input_pcd_topic);

  bool ok() const
  {
    return _ok && ros::ok();
  }

private:

  std::mutex change_input_mtx; // Assures ptr to input cloud won't be changed
                               // while work relying on it is being done

  // Flags
  bool _work_to_do = false;  // Unprocessed PCD
  bool _active = false;      // Activation status
  bool _ok = false;          // Error flag
  std::string _err_msg;

  ros::NodeHandle _nh;
  size_t _img_hist_size{10};  // All camera streams will keep this many frames 
                              // in their buffers
  SingleCloudProcessor _cloud_processor;

  // Subscribing and storing input
  std::string _input_pcd_topic;
  ros::Subscriber _cloud_sub;
  PCD<>::ConstPtr _current_color_pcd;  // Template default argument is in common.hpp 

  // Storing result and publishing
  std::string _output_pcd_topic;
  ros::Publisher _cloud_pub;
  PCD<>::ConstPtr _output_pcd;

  // SPtrVector<CamStream> _ros_cam_ptrs;
  // SPtrVector<PointCloud> _transformed_cloud_ptrs;
  // SPtrVector<ColorObservation::VecImg> _obs_vec_img_ptrs;
  // PCD<pcl::PointXYZRGBA> color_permanence_pcd;


  void _cloud_cb(const PCD<>::ConstPtr &cloud_in);
  // void _process_pcd(const PointCloud::ConstPtr &cloud_in);
  // void _combine_pcd();
  // tf::TransformListener _tf_listener;

}; // end class PcdColorizer

} // namespace nav
