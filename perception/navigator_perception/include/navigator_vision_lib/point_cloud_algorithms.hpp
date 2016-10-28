#include <ros/ros.h>
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
#include <tf/transform_listener.h>
#include <sensor_msgs/image_encodings.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/point_cloud2_iterator.h>
#include <pcl_ros/point_cloud.h>
#include <pcl_ros/transforms.h>

namespace nav{

class PcdSubPubAlgorithm{
  /*
    virtual base class for algorithms that subscribe to point cloud ROS topics,
    operate on the clouds and publish output clouds to a different topic
  */
  typedef sensor_msgs::PointCloud2 PCD;

public:
  PcdSubPubAlgorithm(){};
  ~PcdSubPubAlgorithm(){};
  
protected:
  virtual void cloud_cb(const PCD &cloud_msg ) = 0;  // runs algorithm pipeline when a msg is received
  PCD input_pcd;
  PCD output_pcd;
  ros::NodeHandle nh;
  ros::Subscriber cloud_sub;
  ros::Publisher cloud_pub;
  std::string input_pcd_topic;
  std::string output_pcd_topic;
};


class PcdColorizer : public PcdSubPubAlgorithm{
  /*
    This class takes adds color information to XYZ only point clouds if the
    points in the cloud can be observed by a camera that takes color images
    Note: needs accurate TF because it uses the ros tf package to transform 
    arbitrary point cloud frames into the frame of the color pinhole camera
    Note: the rgb camera topic should be streaming rectified images
  */

  typedef sensor_msgs::PointCloud2 PCD;

public:
  PcdColorizer(ros::NodeHandle nh, std::string input_pcd_topic, std::string output_pcd_topic, std::string rgb_cam_topic, std::string rgb_cam_frame);
  ~PcdColorizer(){}
  void _transform_to_cam();
  void _color_pcd();  

private:
  void cloud_cb(const sensor_msgs::PointCloud2 &cloud_msg);
  std::string rgb_cam_frame;
  std::string rgb_cam_topic;
  tf::TransformListener tf_listener;
  PCD transformed_pcd; // input pcd transformed to the frame of the rgb camera
  image_transport::ImageTransport img_transport {nh};
  image_transport::CameraSubscriber rgb_cam_sub;

 int seq = 0;

  Eigen::Matrix3f cam_intrinsics;
  bool _intrinsics_set = false;
  sensor_msgs::ImageConstPtr latest_frame_img_msg;
  sensor_msgs::CameraInfoConstPtr latest_frame_info_msg;
};

} // namwspace nav