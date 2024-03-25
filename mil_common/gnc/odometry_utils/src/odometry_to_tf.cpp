#include <nav_msgs/Odometry.h>
#include <tf2/transform_datatypes.h>

#include <map>
#include <string>

#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

namespace odometry_utils
{
class odometry_to_tf : public rclcpp::Node
{
private:
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub;
  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_br_;
  std::map<std::string, rclcpp::Time> _last_tf_stamps;

  void handle_odom(const nav_msgs::msg::Odometry::ConstPtr& msg)
  {
    geometry_msgs::msg::TransformStamped stamped_transform;

    stamped_transform.header.stamp = msg->header.stamp;
    stamped_transform.header.frame_id = msg->header.frame_id;
    stamped_transform.child_frame_id = msg->child_frame_id;

    stamped_transform.transform.translation.x = msg->pose.pose.position.x;
    stamped_transform.transform.translation.y = msg->pose.pose.position.y;
    stamped_transform.transform.translation.z = msg->pose.pose.position.z;
    stamped_transform.transform.rotation = msg->pose.pose.orientation;

    if (_last_tf_stamps.count(msg->header.frame_id) && _last_tf_stamps[msg->header.frame_id] == msg->header.stamp)
    {
      return;
    }
    _last_tf_stamps[msg->header.frame_id] = msg->header.stamp;

    tf_br_->sendTransform(stamped_transform);
  }

public:
  odometry_to_tf()
  {
  }

  virtual void onInit()
  {
    tf_br_ = std::make_unique<tf2_ros::TransformBroadcaster>(this);
    odom_sub = this->create_subscription<nav_msgs::msg::Odometry>("odom", 10, std::bind(&OdometryToTf::handle_odom, this, std::placeholders::_1));

  }
};

}
#include <rclcpp_components/register_node_macro.hpp> // Include macro to register the component
RCLCPP_COMPONENTS_REGISTER_NODE(odometry_utils::OdometryToTf) // Register the node as a component
