#include <nav_msgs/Odometry.h>
#include <nodelet/nodelet.h>
#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_datatypes.h>

#include <map>
#include <pluginlib/class_list_macros.hpp>
#include <string>

namespace odometry_utils
{
class odometry_to_tf : public nodelet::Nodelet
{
private:
  ros::Subscriber odom_sub;
  tf::TransformBroadcaster tf_br;
  std::map<std::string, ros::Time> _last_tf_stamps;

  void handle_odom(const nav_msgs::Odometry::ConstPtr& msg)
  {
    tf::Transform transform;
    poseMsgToTF(msg->pose.pose, transform);
    if (_last_tf_stamps.count(msg->header.frame_id) && _last_tf_stamps[msg->header.frame_id] <= msg->header.stamp)
    {
      return;
    }
    _last_tf_stamps[msg->header.frame_id] = msg->header.stamp;
    tf::StampedTransform stamped_transform(transform, msg->header.stamp, msg->header.frame_id, msg->child_frame_id);
    tf_br.sendTransform(stamped_transform);
  }

public:
  odometry_to_tf()
  {
  }

  virtual void onInit()
  {
    odom_sub =
        getNodeHandle().subscribe<nav_msgs::Odometry>("odom", 10, boost::bind(&odometry_to_tf::handle_odom, this, _1));
  }
};

PLUGINLIB_EXPORT_CLASS(odometry_utils::odometry_to_tf, nodelet::Nodelet);
}  // namespace odometry_utils
