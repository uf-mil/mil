#include <eigen_conversions/eigen_msg.h>
#include <odom_estimator/odometry.h>
#include <odom_estimator/unscented_transform.h>
#include <odom_estimator/util.h>
#include <rclcpp/rclcpp.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/convert.h>
#include <tf2/transform_datatypes.h>
#include <tf2_eigen/tf2_eigen.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

#include <nav_msgs/msg/odometry.hpp>
#include <pluginlib/class_list_macros.hpp>
#include <rclcpp/node.hpp>

namespace odometry_utils
{
using namespace odom_estimator;

using namespace Eigen;

class transform_odometry : public rclcpp::Node
{
private:
  std::string frame_id;
  std::string child_frame_id;

  tf2_ros::Buffer tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr sub;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr pub;

  void handle(const nav_msgs::msg::Odometry::SharedPtr msgp)
  {
    geometry_msgs::msg::TransformStamped lefttransform;
    try
    {
      lefttransform = tf_buffer_.lookupTransform(frame_id, msgp->header.frame_id, tf2::TimePointZero);
    }
    catch (tf2::TransformException &ex)
    {
      RCLCPP_ERROR_THROTTLE(this->get_logger(), *this->get_clock(), 10.0, "%s", ex.what());
      return;
    }

    geometry_msgs::msg::TransformStamped righttransform;
    try
    {
      righttransform = tf_buffer_.lookupTransform(msgp->child_frame_id, child_frame_id, tf2::TimePointZero);
    }
    catch (tf2::TransformException &ex)
    {
      RCLCPP_ERROR_THROTTLE(this->get_logger(), *this->get_clock(), 10.0, "%s", ex.what());
      return;
    }

    Eigen::Vector3d left_p;
    tf2::fromMsg(transformStamped.transform.translation, left_p);
    Eigen::Quaterniond left_q;
    tf2::fromMsg(transformStamped.transform.rotation, left_q);

    Eigen::Vector3d right_p;
    tf2::fromMsg(righttransform.transform.translation, right_p);
    Eigen::Quaterniond right_q;
    tf2::fromMsg(righttransform.transform.rotation, right_q);

    EasyDistributionFunction<Odom, Odom, Vec<0> > transformer(
        [this, &left_p, &left_q, &right_p, &right_q](Odom const &odom, Vec<0> const &extra)
        {
          return Odom(odom.stamp, frame_id, child_frame_id, left_p + left_q * (odom.pos + odom.orient * right_p),
                      left_q * odom.orient * right_q, right_q.inverse() * (odom.vel - right_p.cross(odom.ang_vel)),
                      right_q.inverse() * odom.ang_vel);
        },
        GaussianDistribution<Vec<0> >(Vec<0>(), SqMat<0>()));

    pub.publish(msg_from_odom(transformer(odom_from_msg(*msgp))));
  }

public:
  transform_odometry()
    : Node("transform_odometry")
    , tf_buffer_(this->get_clock())
    , tf_listener_(std::make_shared<tf2_ros::TransformListener>(tf_buffer_))
  {
    this->declare_parameter<std::string>("frame_id", "default_frame_id");              // Default value as example
    this->declare_parameter<std::string>("child_frame_id", "default_child_frame_id");  // Default value as example

    if (!this->get_parameter("frame_id", frame_id))
    {
      throw std::runtime_error("param 'frame_id' required");
    }
    if (!this->get_parameter("child_frame_id", child_frame_id))
    {
      throw std::runtime_error("param 'child_frame_id' required");
    }

    sub = this->create_subscription<nav_msgs::msg::Odometry>(
        "orig_odom", 10, std::bind(&TransformOdometry::handle, this, std::placeholders::_1));

    pub = this->create_publisher<nav_msgs::msg::Odometry>("odom", 10);
  };

#include <rclcpp_components/register_node_macro.hpp>
  RCLCPP_COMPONENTS_REGISTER_NODE(odometry_utils::TransformOdometry)
}
