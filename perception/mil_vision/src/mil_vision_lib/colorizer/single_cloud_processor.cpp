#include <colorizer/single_cloud_processor.hpp>

// using namespace std;

namespace nav {

using nav::tools::operator "" _s; // converts to std::string

SingleCloudProcessor::SingleCloudProcessor(ros::NodeHandle nh, std::string& in_pcd_topic, size_t hist_size)
: _nh{nh}, _hist_size{hist_size}
{
  ROS_INFO(("SingleCloudProcessor: Initializing with " + in_pcd_topic).c_str());
  auto rect_color_topics = nav::tools::getRectifiedImageTopics(true);
  if(rect_color_topics.size() == 0)
  {
    _err_msg += "COLORIZER: There are no rectified color camera topics currently publishing on this ROS master (";
    _err_msg += ros::master::getURI();
    _err_msg += ") Re-run node after rectified color images are being published.";
    ROS_ERROR(_err_msg.c_str());
    return;
  }

  for(auto& topic : rect_color_topics)
  {
    ROS_INFO(("SingleCloudProcessor: Creating CameraObserver for camera publishing to "_s + topic.c_str()).c_str());
    auto cam_observer_ptr = new CameraObserver{_nh, in_pcd_topic, topic, _hist_size};
    ROS_INFO(("CameraObserver: initialization "_s + (cam_observer_ptr->ok()? "successful" : "unsuccessful")).c_str());
    if(cam_observer_ptr->ok())
      _camera_observers.push_back(std::unique_ptr<CameraObserver>{cam_observer_ptr});
    else
      delete cam_observer_ptr;
  }

  if(_camera_observers.size() == 0)
  {
    _err_msg = "SingleCloudProcessor: No ROSCameraStreams could be initialized.";
    ROS_ERROR_NAMED("COLORIZER", _err_msg.c_str());
  }
  else
    _ok = true;
  return;
}

void SingleCloudProcessor::operator()(const PCD<pcl::PointXYZ>::ConstPtr &pcd)
{

  std::cout << "Called the single cloud processor operator()" << std::endl;
  std::cout << "pcd: " << pcd->header << std::endl;


  // Get color observation list from each of the observers

  // Merge lists from all of the observers

  // Summarize Confidence in each point color observation
}

}  // namespace nav
