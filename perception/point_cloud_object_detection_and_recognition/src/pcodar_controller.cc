#include <point_cloud_object_detection_and_recognition/pcodar_controller.hh>

#include <mil_msgs/PerceptionObject.h>
#include <mil_msgs/PerceptionObjectArray.h>
#include <nav_msgs/OccupancyGrid.h>
#include <pcl_ros/transforms.h>

#include <pcl/features/normal_3d.h>
#include <pcl/kdtree/kdtree.h>
#include <pcl/point_cloud.h>
#include <pcl/registration/icp.h>
#include <pcl/segmentation/extract_clusters.h>

#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>

#include <pcl_ros/point_cloud.h>

#include <tf2/convert.h>
#include <tf2_msgs/TFMessage.h>
#include <tf2_ros/transform_listener.h>

#include <eigen_conversions/eigen_msg.h>

#include <ros/callback_queue.h>
#include <ros/console.h>

#include <chrono>
#include <opencv2/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>

namespace pcodar
{
ros::NodeHandle init(int argc, char *argv[], pcodar_params &params)
{
    ros::init(argc, argv, "point_cloud_object_detector");
    ros::Time::init();
    auto nh = ros::NodeHandle(ros::this_node::getName());
    set_params(nh, params);

    // Node handler
    return nh;
}

pcodar_controller::pcodar_controller(int argc, char *argv[])
    : nh_(init(argc, argv, params_)), detector_(params_)
{
    id_object_map temp_map;
}
void pcodar_controller::initialize()
{
    marker_manager_.initialize(nh_);

    // Subscribe to odom and the velodyne
    pc_sub = nh_.subscribe("/velodyne_points", 1, &pcodar_controller::velodyne_cb, this);
    odom_sub = nh_.subscribe("/odom", 1, &pcodar_controller::odom_cb, this);

    // Publish occupancy grid and visualization markers
    pub_grid_ = nh_.advertise<nav_msgs::OccupancyGrid>("/pcodar/ogrid", 10);
    pub_pcl_ = nh_.advertise<point_cloud>("/pcodar/persist_pcl", 1);

    // Publish PerceptionObjects
    pub_objects_ = nh_.advertise<mil_msgs::PerceptionObjectArray>("/pcodar/objects", 1);
}

void pcodar_controller::velodyne_cb(const sensor_msgs::PointCloud2ConstPtr &pcloud) { latest_point_cloud_ = *pcloud; }
void pcodar_controller::odom_cb(const nav_msgs::OdometryConstPtr &odom) { latest_odom_ = odom; }
void pcodar_controller::execute()
{
    ros::Rate r(params_.executive_rate);
    while (ros::ok())
    {
        // Execute all callbacks
        ros::spinOnce();

        // Do work
        executive();

        // Wait
        r.sleep();
    }
}

void pcodar_controller::executive()
{
    // Checking to see if a point cloud has been set yet. This is done by checking the height. This is due to the
    // following:
    // The point cloud can be store in two ways
    // - In image form (think depth map) which means the width and height will match the size of the image
    // - In unordered form, which means height will be 1 and width will be the number of points (This is what is
    // used
    // for lidar point clouds)
    // In both of these the height is 1 or greater. So this is what is checked here.
    if (latest_point_cloud_.height < 1)
    {
        return;
    }

    // Use ROS transform listener to grad up-to-date transforms between reference frames
    static tf2_ros::Buffer tfBuffer;
    static tf2_ros::TransformListener tf_listener(tfBuffer);
    geometry_msgs::TransformStamped T_enu_velodyne_ros;
    try
    {
        T_enu_velodyne_ros = tfBuffer.lookupTransform(
            "enu", "velodyne", ros::Time(0));  // change time to pcloud header? pcloud->header.stamp
    }
    catch (tf2::TransformException &ex)
    {
        ROS_ERROR("%s", ex.what());
        return;
    }

    Eigen::Affine3d e_transform;
    tf::transformMsgToEigen(T_enu_velodyne_ros.transform, e_transform);
    detector_.add_point_cloud(latest_point_cloud_, e_transform);

    auto objects = detector_.get_objects();
    marker_manager_.update_markers(objects.objects);

    pub_objects_.publish(objects);

}
}  // namespace pcodar