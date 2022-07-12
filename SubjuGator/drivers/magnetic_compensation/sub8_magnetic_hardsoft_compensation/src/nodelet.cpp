#include <eigen_conversions/eigen_msg.h>
#include <nodelet/nodelet.h>
#include <ros/ros.h>
#include <sensor_msgs/MagneticField.h>

#include <boost/shared_ptr.hpp>
#include <boost/thread.hpp>
#include <mil_tools/param_helpers.hpp>
#include <pluginlib/class_list_macros.hpp>

namespace magnetic_hardsoft_compensation
{
class Nodelet : public nodelet::Nodelet
{
private:
  std::string frame_id;
  Eigen::Matrix3d scale_inverse;
  Eigen::Vector3d shift;
  ros::Subscriber sub;
  ros::Publisher pub;

  void handle(const sensor_msgs::MagneticField::ConstPtr& msg)
  {
    if (msg->header.frame_id != frame_id)
    {
      ROS_ERROR("msg's frame_id != configured frame_id! ignoring message");
      return;
    }

    Eigen::Vector3d raw;
    tf::vectorMsgToEigen(msg->magnetic_field, raw);

    Eigen::Vector3d processed = scale_inverse * (raw - shift);

    sensor_msgs::MagneticField result;
    result.header = msg->header;
    tf::vectorEigenToMsg(processed, result.magnetic_field);

    pub.publish(result);
  }

public:
  Nodelet()
  {
  }

  virtual void onInit()
  {
    ros::NodeHandle& private_nh = getPrivateNodeHandle();

    frame_id = mil_tools::getParam<std::string>(private_nh, "frame_id");
    scale_inverse = mil_tools::getParam<Eigen::Matrix3d>(private_nh, "scale").inverse();
    shift = mil_tools::getParam<Eigen::Vector3d>(private_nh, "shift");

    ros::NodeHandle& nh = getNodeHandle();

    sub = nh.subscribe<sensor_msgs::MagneticField>("imu/mag_raw", 1000, boost::bind(&Nodelet::handle, this, _1));
    pub = nh.advertise<sensor_msgs::MagneticField>("imu/mag", 10);
  }
};

PLUGINLIB_EXPORT_CLASS(magnetic_hardsoft_compensation::Nodelet, nodelet::Nodelet);
}  // namespace magnetic_hardsoft_compensation
