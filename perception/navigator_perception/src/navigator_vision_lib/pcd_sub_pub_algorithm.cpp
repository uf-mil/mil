#include <navigator_vision_lib/pcd_sub_pub_algorithm.hpp>

namespace nav{

PcdSubPubAlgorithm::PcdSubPubAlgorithm(ros::NodeHandle nh, std::string input_pcd_topic, std::string output_pcd_topic)
: _nh(nh), _input_pcd_topic(input_pcd_topic), _output_pcd_topic(output_pcd_topic)
{
  // Subscribe to point cloud topic
  _cloud_sub = _nh.subscribe<PointCloud>(_input_pcd_topic, 1, &PcdSubPubAlgorithm::cloud_cb, this);

  // Advertise output topic
  _cloud_pub = _nh.advertise<PointCloud>(_output_pcd_topic, 1, true);
}

}  // namespace nav