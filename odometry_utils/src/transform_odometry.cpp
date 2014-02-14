#include <nodelet/nodelet.h>
#include <pluginlib/class_list_macros.h>
#include <ros/ros.h>
#include <tf/LinearMath/Quaternion.h>
#include <tf/transform_datatypes.h>
#include <tf/transform_listener.h>
#include <tf_conversions/tf_eigen.h>
#include <eigen_conversions/eigen_msg.h>

#include <odom_estimator/util.h>
#include <odom_estimator/unscented_transform.h>
#include <odom_estimator/odometry.h>

namespace odometry_utils {

using namespace odom_estimator;

using namespace Eigen;


class transform_odometry : public nodelet::Nodelet {
private:
  std::string frame_id;
  std::string child_frame_id;
  
  tf::TransformListener listener;
  
  ros::Subscriber sub;
  ros::Publisher pub;
  
  void handle(const nav_msgs::Odometry::ConstPtr& msgp) {
    tf::StampedTransform lefttransform;
    try {
      listener.lookupTransform(frame_id, msgp->header.frame_id,
        ros::Time(0), lefttransform);
    } catch (tf::TransformException ex) {
      ROS_ERROR("%s", ex.what());
      return;
    }
    
    tf::StampedTransform righttransform;
    try {
      listener.lookupTransform(msgp->child_frame_id, child_frame_id,
        ros::Time(0), righttransform);
    } catch (tf::TransformException ex) {
      ROS_ERROR("%s", ex.what());
      return;
    }
    
    Vector3d left_p; tf::vectorTFToEigen(lefttransform.getOrigin(), left_p);
    Quaterniond left_q; tf::quaternionTFToEigen(lefttransform.getRotation(), left_q);
    Vector3d right_p; tf::vectorTFToEigen(righttransform.getOrigin(), right_p);
    Quaterniond right_q; tf::quaternionTFToEigen(righttransform.getRotation(), right_q);
    
    EasyDistributionFunction<Odom, Odom, Vec<0> > transformer(
      [&left_p, &left_q, &right_p, &right_q](Odom const &odom,
                                             Vec<0> const &extra) {
        return Odom(
          odom.stamp,
          odom.frame_id,
          odom.child_frame_id,
          left_p + left_q._transformVector(odom.pos + odom.orient._transformVector(right_p)),
          left_q * odom.orient * right_q,
          right_q.inverse()._transformVector(odom.vel - right_p.cross(odom.ang_vel)),
          right_q.inverse()._transformVector(odom.ang_vel));
      },
      GaussianDistribution<Vec<0> >(Vec<0>(), SqMat<0>()));
    
    pub.publish(msg_from_odom(transformer(odom_from_msg(*msgp))));
  }

public:
  transform_odometry() {}
  
  virtual void onInit() {
    if(!getPrivateNodeHandle().getParam("frame_id", frame_id)) {
      throw std::runtime_error("param frame_id required");
    }
    if(!getPrivateNodeHandle().getParam("child_frame_id", child_frame_id)) {
      throw std::runtime_error("param child_frame_id required");
    }
    
    sub = getNodeHandle().subscribe<nav_msgs::Odometry>("orig_odom", 10,
      boost::bind(&transform_odometry::handle, this, _1));
    pub = getNodeHandle().advertise<nav_msgs::Odometry>("odom", 10);
  }
};
PLUGINLIB_DECLARE_CLASS(odometry_utils, transform_odometry,
  odometry_utils::transform_odometry, nodelet::Nodelet);

}
