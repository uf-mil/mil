#include <colorizer/color_observation.hpp>


namespace nav{

OcclusionImg::OcclusionImg(boost::shared_ptr<pcl::PointCloud<pcl::PointXYZ>> cloud_ptr, ROSCameraStream<cv::Vec3b>::CamFramePtr &frame_ptr, float dist_thresh)
: frame_ptr(frame_ptr),
  cloud_ptr(cloud_ptr)  
{
  auto cam_model = frame_ptr->getCameraModelPtr();


}

} // namespace nav