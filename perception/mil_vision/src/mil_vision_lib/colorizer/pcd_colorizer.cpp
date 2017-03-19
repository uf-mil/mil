#include <mil_vision_lib/colorizer/pcd_colorizer.hpp>

using namespace std;
using namespace cv;

namespace nav{

PcdColorizer::PcdColorizer(ros::NodeHandle nh, string input_pcd_topic)
  : _nh(nh), _img_hist_size{10}, _cloud_processor{nh, input_pcd_topic, _img_hist_size},
  _input_pcd_topic{input_pcd_topic}, _output_pcd_topic{input_pcd_topic + "_colored"}  
{
  using mil::tools::operator "" _s; // converts to std::string

  try
  {
    // Subscribe to point cloud topic
    _cloud_sub = _nh.subscribe<PCD<>>(_input_pcd_topic, 1, &SingleCloudProcessor::operator(), &_cloud_processor);

    // Advertise output topic
    _cloud_pub = _nh.advertise<PCD<>>(_output_pcd_topic, 1, true); // output type will probably be different
  }
  catch(std::exception &e)
  {
    _err_msg = "COLORIZER: Suscriber or publisher error caught: "_s + e.what();
    ROS_ERROR(_err_msg.c_str());
    return;
  }

  std::string msg = "COLORIZER: Initialization was "_s + (_cloud_processor.ok()? "successful" : "unsuccessful");
  msg += "\n    Input point cloud topic: " + _input_pcd_topic + "\n    Output point cloud topic: " + _output_pcd_topic;
  ROS_INFO(msg.c_str());

  if(_cloud_processor.ok())
  {
    _ok = true;
    _active = true;
  }
}

void PcdColorizer::_cloud_cb(const PCD<>::ConstPtr &cloud_in)
{
  change_input_mtx.lock();
  // _input_pcd = cloud_in;  // TODO: figure out a good communication mechanism b/w colorizer, single processor and merger
  ROS_INFO("Got a new cloud");
  change_input_mtx.unlock();
}

// PcdColorizer::PcdColorizer(ros::NodeHandle nh, string input_pcd_topic)
//   : PcdSubPubAlgorithm{nh, input_pcd_topic, input_pcd_topic + "_colored"},
//   _img_hist_size{10},
//   cloud_processor{nh, _img_hist_size}
// {
//   cout << "img hist size: " << _img_hist_size << endl;
//   // Subscribe to all rectified color img topics
//   auto rect_color_topics = mil::tools::getRectifiedImageTopics(true);
//   if(rect_color_topics.size() == 0)
//   {
//     _err_msg += "COLORIZER: There are no rectified color camera topics currently publishing on this ROS master (";
//     _err_msg += ros::master::getURI();
//     _err_msg += ") Re-run after rectified color images are being published.";
//     ROS_ERROR(_err_msg.c_str());
//     return;
//   } 

//   // Construct and initialize Camera Stream objects
//   size_t good_init = 0;
//   for(size_t i = 0; i < rect_color_topics.size(); i++)
//   {
//     // Construct on heap but store ptrs in stack
//     _ros_cam_ptrs.emplace_back(new CamStream{this->_nh, this->_img_hist_size});
//     good_init += int(_ros_cam_ptrs[i]->init(rect_color_topics[i]));  // Count successfull init
//     _transformed_cloud_ptrs.emplace_back(nullptr);
//   }

//   // Make sure at least one ROSCameraStream was successfully initialized
//   if(good_init == 0)
//   {
//     _err_msg = "COLORIZER: No ROSCameraStreams could be initialized.";
//     ROS_ERROR_NAMED("COLORIZER", _err_msg.c_str());
//     return;
//   }

//   std::mutex mtx;

//   // LAMBDAS!
//   auto check_tf_lambda = [this, &rect_color_topics, &mtx](const PointCloud::ConstPtr &cloud_in)
//     {
//       std::lock_guard<std::mutex> lock{mtx};  // will unlock mtx when it goes out of scope
//       ROS_INFO("COLORIZER: checking that all required TF's are available");
//       std::string src_frame_id = cloud_in->header.frame_id;
//       std::string target_frame_id;
//       ros::Duration tf_timeout{5, 0}; // Wait 5 seconds max for each TF
//       this->_ok = true;  // assume TF's are ok until proven otherwise

//       // Check each TF
//       for(size_t i = 0; i < rect_color_topics.size(); i++)
//       {
//         std::string err {"COLORIZER: waiting for tf between "};
//         err += src_frame_id + std::string(" and ") + target_frame_id + std::string(": ");
//         if(! this->_tf_listener.waitForTransform(this->_ros_cam_ptrs[i]->getCameraModelPtr()->tfFrame(), 
//                                                  src_frame_id, ros::Time(0), tf_timeout, ros::Duration(0.05), &err))
//         {
//           ROS_ERROR(err.c_str());  // TF not available
//           this->_ok = false;
//         }
//       }
//       if(this->_ok)
//         ROS_INFO("COLORIZER: Required TF's are publishing.");
//       mtx.unlock();
//     };

// void PcdColorizer::_cloud_cb(const PCD<>::ConstPtr &cloud_in)
// {
//   std::cout << "active: " << _active << std::endl;
//   std::cout << "fuck!" << std::endl;
//   if(_active)
//   {
//     if(!_ok)
//     {
//       ROS_INFO((std::string("COLORIZER: received cloud msg but error flag is set: ") + _err_msg).c_str());
//       return;
//     }

//     // std::cout << "fuck!" << std::endl;

//     // // Transforms pcd to frame of each camera and generates list of color observations for points
//     // // observed in any of the cameras
//     // _process_pcd(cloud_in);

//     // // Combines current colored point cloud with stored persistent color pcd
//     // _combine_pcd();
//   }
//   else
//     ROS_WARN_DELAYED_THROTTLE(15, "COLORIZER: receiving clouds but not active");
//   std::cout << "Exiting " << __PRETTY_FUNCTION__ << std::endl;
// }

// inline void PcdColorizer::_process_pcd(const PCD<>::ConstPtr &cloud_in)
// {
//   std::cout << "process cloud" << std::endl;
//   // Transform pcds to target tf frames
//   std::string target_frame_id;

//   // Synchronize work for each camera among worker threads
//   std::promise<void> start_work_prom;
//   std::shared_future<void> star_work_fut{start_work_prom.get_future()};
//   std::vector<std::promise<void>> worker_done_proms{_ros_cam_ptrs.size()};
//   std::vector<std::future<void>> worker_done_futs{_ros_cam_ptrs.size()};

//   #pragma omp parallel for
//   for(size_t i = 0; i < _ros_cam_ptrs.size(); i++)
//   {
//     target_frame_id = _ros_cam_ptrs[i]->getCameraModelPtr()->tfFrame();
//     std::shared_ptr<PointCloud> transfromed_pcd{new PointCloud()};
//     pcl_ros::transformPointCloud<pcl::PointXYZ>(target_frame_id, *cloud_in, *transfromed_pcd, _tf_listener);
//     _transformed_cloud_ptrs[i] = transfromed_pcd;
//   }

  // #pragma omp parallel for
  // for(size_t i = 0; i < _transformed_cloud_ptrs.size(); i++)
  // {
  //   // This should be merged with above loop
  //   std::cout << "hey" << std::endl;
  // }
  // std::cout << "pcd: " <<  cloud_in->header << std::endl;
// }

// inline void PcdColorizer::_combine_pcd()
// {

// }

} // namespace nav
