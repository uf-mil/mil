#include <nodelet/nodelet.h>
#include <pluginlib/class_list_macros.h>
#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <tf/LinearMath/Quaternion.h>
#include <tf/transform_datatypes.h>
#include <tf/transform_listener.h>

#include <uf_common/param_helpers.h>


namespace odometry_utils {
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
                    listener.lookupTransform(frame_id, msg->header.frame_id, ros::Time(0), lefttransform);
                } catch (tf::TransformException ex) {
                    ROS_ERROR("%s", ex.what());
                    return;
                }
                
                
                tf::StampedTransform righttransform;
                try {
                    listener.lookupTransform(msg->child_frame_id, child_frame_id, ros::Time(0), righttransform);
                } catch (tf::TransformException ex) {
                    ROS_ERROR("%s", ex.what());
                    return;
                }
                
                // XXX do covariance
                
                nav_msgs::Odometry result;
                result.header.frame_id = frame_id;
                result.header.stamp = msg->header.stamp;
                result.child_frame_id = child_frame_id;
                
                tf::Transform transform; poseMsgToTF(msg->pose.pose, transform);
                tf::Transform pose = lefttransform * transform * righttransform;
                poseTFToMsg(pose, result.pose.pose);
                
                tf::Vector3 w; vector3MsgToTF(msg->twist.twist.angular, w);
                vector3TFToMsg(quatRotate(righttransform.getRotation().inverse(), w), result.twist.twist.angular);
                
                tf::Vector3 v; vector3MsgToTF(msg->twist.twist.linear, v);
                tf::Vector3 v2 = v - righttransform.getOrigin().cross(w);
                vector3TFToMsg(quatRotate(righttransform.getRotation().inverse(), v2), result.twist.twist.linear);
                
                
                pub.publish(result);
            }
        
        public:
            transform_odometry() {}
            
            virtual void onInit() {
                ROS_ASSERT(getPrivateNodeHandle().getParam("frame_id", frame_id));
                ROS_ASSERT(getPrivateNodeHandle().getParam("child_frame_id", child_frame_id));
                
                sub = getNodeHandle().subscribe<nav_msgs::Odometry>("orig_odom", 10, boost::bind(&transform_odometry::handle, this, _1));
                pub = getNodeHandle().advertise<nav_msgs::Odometry>("odom", 10);
            }

    };

    PLUGINLIB_DECLARE_CLASS(odometry_utils, transform_odometry, odometry_utils::transform_odometry, nodelet::Nodelet);
}
