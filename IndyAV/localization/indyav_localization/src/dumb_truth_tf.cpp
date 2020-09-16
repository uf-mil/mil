#include <nav_msgs/Odometry.h>
#include <ros/ros.h>
#include <tf/transform_broadcaster.h>

// TODO: remove this code when p3d in gazebo is replaced with the sylphase xacro

void cb(const nav_msgs::Odometry& _msg)
{
  static tf::TransformBroadcaster br;
  tf::Transform transform;
  transform.setOrigin(tf::Vector3(_msg.pose.pose.position.x, _msg.pose.pose.position.y, _msg.pose.pose.position.z));
  transform.setRotation(tf::Quaternion(_msg.pose.pose.orientation.x, _msg.pose.pose.orientation.y,
                                       _msg.pose.pose.orientation.z, _msg.pose.pose.orientation.w));
  br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), _msg.header.frame_id, _msg.child_frame_id));
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "dumb_truth_odom_tf");
  ros::NodeHandle nh("~");
  std::string topic_name;
  ROS_ASSERT_MSG(nh.getParam("topic_name", topic_name), "no topic name provided");
  ros::Subscriber sub = nh.subscribe(topic_name, 1, cb);

  ros::spin();

  return 0;
}
