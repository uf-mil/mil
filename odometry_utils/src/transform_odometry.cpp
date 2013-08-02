#include <nodelet/nodelet.h>
#include <pluginlib/class_list_macros.h>
#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <tf/LinearMath/Quaternion.h>
#include <tf/transform_datatypes.h>
#include <tf/transform_listener.h>
#include <tf_conversions/tf_eigen.h>
#include <eigen_conversions/eigen_msg.h>

#include <odom_estimator/util.h>
#include <odom_estimator/unscented_transform.h>

namespace odometry_utils {

using namespace Eigen;
typedef Matrix<double, 6, 6> Matrix6d;
typedef Matrix<double, 12, 12> Matrix12d;

struct Odom {
  Vector3d pos;
  Quaterniond orient;
  Vector3d vel;
  Vector3d ang_vel;
  static const int DELTA_SIZE = 3*4;
  typedef Matrix<double, DELTA_SIZE, 1> DeltaType;
  
  Odom(Vector3d pos, Quaterniond orient,
      Vector3d vel, Vector3d ang_vel) :
    pos(pos), orient(orient.normalized()),
    vel(vel), ang_vel(ang_vel) { }
  Odom(const geometry_msgs::Pose &pose,
       const geometry_msgs::Twist &twist) {
    tf::pointMsgToEigen(pose.position, pos);
    tf::quaternionMsgToEigen(pose.orientation, orient);
    tf::vectorMsgToEigen(twist.linear, vel);
    tf::vectorMsgToEigen(twist.angular, ang_vel);
  }
  
  DeltaType operator-(const Odom &other) const {
    return (DeltaType() <<
      pos - other.pos,
      rotvec_from_quat(orient * other.orient.conjugate()),
      vel - other.vel,
      ang_vel - other.ang_vel).finished();
  }
  Odom operator+(const DeltaType &other) const {
    return Odom(
      pos + other.segment<3>(0),
      quat_from_rotvec(other.segment<3>(3)) * orient,
      vel + other.segment<3>(6),
      ang_vel + other.segment<3>(9));
  }
  
  Odom transform(const tf::Transform &left,
                 const tf::Transform &right) const {
    Vector3d left_p; tf::vectorTFToEigen(left.getOrigin(), left_p);
    Quaterniond left_q; tf::quaternionTFToEigen(left.getRotation(), left_q);
    Vector3d right_p; tf::vectorTFToEigen(right.getOrigin(), right_p);
    Quaterniond right_q; tf::quaternionTFToEigen(right.getRotation(), right_q);
    
    return Odom(
      left_p + left_q._transformVector(pos + orient._transformVector(right_p)),
      left_q * orient * right_q,
      right_q.inverse()._transformVector(vel - right_p.cross(ang_vel)),
      right_q.inverse()._transformVector(ang_vel));
  }
};

class transform_odometry : public nodelet::Nodelet {
private:
  std::string frame_id;
  std::string child_frame_id;
  
  tf::TransformListener listener;
  
  ros::Subscriber sub;
  ros::Publisher pub;
  
  void handle(const nav_msgs::Odometry::ConstPtr& msg) {
      tf::StampedTransform lefttransform;
      try {
          listener.lookupTransform(frame_id, msg->header.frame_id,
            ros::Time(0), lefttransform);
      } catch (tf::TransformException ex) {
          ROS_ERROR("%s", ex.what());
          return;
      }
      
      tf::StampedTransform righttransform;
      try {
          listener.lookupTransform(msg->child_frame_id, child_frame_id,
            ros::Time(0), righttransform);
      } catch (tf::TransformException ex) {
          ROS_ERROR("%s", ex.what());
          return;
      }
      
      Matrix12d cov = Matrix12d::Zero();
      cov.block<6, 6>(0, 0) = Map<const Matrix6d>(msg->pose.covariance.data());
      cov.block<6, 6>(6, 6) = Map<const Matrix6d>(msg->twist.covariance.data());
      UnscentedTransform<Odom, 12, Odom, 12> res(
        boost::bind(&Odom::transform, _1, lefttransform, righttransform),
        Odom(msg->pose.pose, msg->twist.twist),
        cov);
      
      nav_msgs::Odometry result;
      result.header.frame_id = frame_id;
      result.header.stamp = msg->header.stamp;
      result.child_frame_id = child_frame_id;
      
      tf::pointEigenToMsg(res.mean.pos, result.pose.pose.position);
      tf::quaternionEigenToMsg(res.mean.orient, result.pose.pose.orientation);
      Map<Matrix6d>(result.pose.covariance.data()) = res.cov.block<6, 6>(0, 0);
      
      tf::vectorEigenToMsg(res.mean.vel, result.twist.twist.linear);
      tf::vectorEigenToMsg(res.mean.ang_vel, result.twist.twist.angular);
      Map<Matrix6d>(result.twist.covariance.data()) = res.cov.block<6, 6>(6, 6);
      
      pub.publish(result);
  }

public:
  transform_odometry() {}
  
  virtual void onInit() {
      ROS_ASSERT(getPrivateNodeHandle().getParam("frame_id", frame_id));
      ROS_ASSERT(getPrivateNodeHandle().getParam("child_frame_id", child_frame_id));
      
      sub = getNodeHandle().subscribe<nav_msgs::Odometry>("orig_odom", 10,
        boost::bind(&transform_odometry::handle, this, _1));
      pub = getNodeHandle().advertise<nav_msgs::Odometry>("odom", 10);
  }
};
PLUGINLIB_DECLARE_CLASS(odometry_utils, transform_odometry,
  odometry_utils::transform_odometry, nodelet::Nodelet);

}
