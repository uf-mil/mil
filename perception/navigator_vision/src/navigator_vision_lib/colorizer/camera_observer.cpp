#include <colorizer/camera_observer.hpp>

namespace nav{

using nav::tools::operator "" _s; // converts to std::string
  
CameraObserver::CameraObserver(ros::NodeHandle &nh, std::string &pcd_in_topic, std::string &cam_topic, size_t hist_size)
: _nh{nh}, _cam_stream{nh, hist_size}
{
  try
  {
    if(!_cam_stream.init(cam_topic))
    {
      _err_msg = "COLORIZER: ROSCameraStreams could not be initialized with "_s + cam_topic + "."_s;
      return;
    }
  }
  catch(std::exception &e)
  {
    std::cout << __PRETTY_FUNCTION__ << " exception caught: " << e.what() << std::endl;
  }


  // Check that tf for this camera is up (default template arg is pcl::PointXYZ)
  auto velodyne_msg = ros::topic::waitForMessage<PCD<>>(pcd_in_topic, _nh, ros::Duration{3, 0});
  std::string src_frame_id = velodyne_msg->header.frame_id;
  std::string target_frame_id;
  ros::Duration tf_timeout{5, 0}; // Wait 5 seconds max for each TF
  std::string err = "COLORIZER: waiting for tf between "_s + src_frame_id + " and "_s + target_frame_id + ": "_s;
 if(_tf_listener.waitForTransform(_cam_stream.getCameraModelPtr()->tfFrame(), src_frame_id, ros::Time(0),
                                 tf_timeout, ros::Duration(0.05), &err))
 {
   _ok = true;
   return; // Ideal return point
 }
 else
   ROS_ERROR(err.c_str());  // TF not available
}

ColorObservation::VecImg CameraObserver::get_color_observations(const PCD<pcl::PointXYZ>::ConstPtr &pcd)
{
  using std::vector;
  using std::cout;
  using std::endl;
  cout << __PRETTY_FUNCTION__ << endl;

  // Structre: Image pixels are lists of ColorObservations for the respective pixels in the image that have 
  // pcd points that would be imaged there
  auto obs_img = vector<vector<ColorObservation>>{size_t(_cam_stream.rows()), vector<ColorObservation>{size_t(_cam_stream.cols())}};
  PCD<pcl::PointXYZRGB> pcd_cam{}; // _cam indicates the reference frame
  auto cam_model = _cam_stream.getCameraModelPtr();

  // We will first transform the pcd from lidar frame into the frame of the camera so that we may project 
  // it into the image with just the camera intrinsics
  pcl_ros::transformPointCloud<pcl::PointXYZ>(cam_model->tfFrame(), *pcd, pcd_cam, _tf_listener);

  cv::Matx33d K_cv = cam_model->fullIntrinsicMatrix();
  Eigen::Matrix3d K_eigen;
  cv::cv2eigen(K_cv, K_eigen);

  for(auto pt : pcd_cam)
  {
    cout << "Pt: " << pt << endl;
    auto imaged_pt = K_eigen * Eigen::Matrix<double, 3, 1>{pt.x, pt.y, pt.z};
    cout << "Imaged pt: " << imaged_pt << endl;
    pt = pcl::PointXYZ(imaged_pt[0], imaged_pt[1], imaged_pt[2]);

  }
  return ColorObservation::VecImg();
  
}

}  // namespace nav
