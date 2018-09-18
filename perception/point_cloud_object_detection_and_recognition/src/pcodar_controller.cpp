#include <point_cloud_object_detection_and_recognition/pcodar_controller.hpp>

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
#include <tf2_eigen/tf2_eigen.h>

#include <eigen_conversions/eigen_msg.h>

#include <ros/callback_queue.h>
#include <ros/console.h>

#include <chrono>
#include <functional>
#include <opencv2/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>

namespace pcodar {

pcodar_controller::pcodar_controller(ros::NodeHandle _nh)
    : nh_(_nh),
      bounds_client_("/bounds_server", std::bind(&pcodar_controller::bounds_update_cb, this, std::placeholders::_1)),
      tf_listener(tf_buffer_, nh_) {
  ros::NodeHandle private_nh("~");
  set_params(private_nh);
  id_object_map_ = std::shared_ptr<id_object_map>(new id_object_map);
  id_label_map_= std::shared_ptr<id_label_map>(new id_label_map);
}

void pcodar_controller::initialize() {
  marker_manager_.initialize(nh_, id_label_map_);
  ogrid_manager_.initialize(nh_);

  modify_classification_service_ = nh_.advertiseService(
      "/database/requests", &pcodar_controller::DBQuery_cb, this);

  // Subscribe to odom and the velodyne
  pc_sub = nh_.subscribe("/velodyne_points", 1, &pcodar_controller::velodyne_cb,
                         this);
  odom_sub = nh_.subscribe("/odom", 1, &pcodar_controller::odom_cb, this);

  // Publish occupancy grid and visualization markers
  pub_pcl_ = nh_.advertise<point_cloud>("persist_pcl", 1);

  // Publish PerceptionObjects
  pub_objects_ = nh_.advertise<mil_msgs::PerceptionObjectArray>("objects", 1);
}

void pcodar_controller::velodyne_cb(
    const sensor_msgs::PointCloud2ConstPtr &pcloud) {
  latest_point_cloud_ = *pcloud;
}

void pcodar_controller::odom_cb(const nav_msgs::OdometryConstPtr &odom) {
  latest_odom_ = odom;
}

void pcodar_controller::execute() {
  ros::Rate r(params.executive_rate);
  while (ros::ok()) {
    // Execute all callbacks
    ros::spinOnce();

    // Do work
    executive();

    // Wait
    r.sleep();
  }
}

void pcodar_controller::executive() {
  // Checking to see if a point cloud has been set yet. This is done by checking
  // the height. This is due to the
  // following:
  // The point cloud can be store in two ways
  // - In image form (think depth map) which means the width and height will
  // match the size of the image
  // - In unordered form, which means height will be 1 and width will be the
  // number of points (This is what is
  // used
  // for lidar point clouds)
  // In both of these the height is 1 or greater. So this is what is checked
  // here.
  if (latest_point_cloud_.height < 1) {
    return;
  }

  geometry_msgs::TransformStamped T_enu_velodyne_ros;
  try {
    T_enu_velodyne_ros = tf_buffer_.lookupTransform(
        "enu", latest_point_cloud_.header.frame_id,
        latest_point_cloud_.header.stamp,
        ros::Duration(1, 0));  // change time to pcloud header? pcloud->header.stamp
  } catch (tf2::TransformException &ex) {
    ROS_ERROR("%s", ex.what());
    return;
  }

  Eigen::Affine3d e_transform;
  tf::transformMsgToEigen(T_enu_velodyne_ros.transform, e_transform);
  detector_.add_point_cloud(latest_point_cloud_, e_transform);

  auto objects =
      detector_.get_objects(pub_pcl_);
  if (id_object_map_->empty()) {
    for (auto &object : objects->objects) {
      id_object_map_->insert({object.id, object});
    }
  }

  std::vector<association_unit> association_unit =
      ass.associate(*id_object_map_, objects->objects);

  try {
    for (const auto &a : association_unit) {
      //         // std::cout << a.first <<endl;
      auto w = id_object_map_->find(a.object_id);
      auto z = objects->objects.at(a.index);

      if (w == id_object_map_->end()) {
        // std::cout <<a.index << " " << a.object_id << " does not exist" <<
        // id_object_map_->size() << std::endl;
        id_object_map_->insert({a.index, z});  // TODO
        continue;
      }
      z.id = w->first;
      auto cl = w->second.classification;
      auto cll = w->second.labeled_classification;
      w->second = z;
      w->second.classification = cl;
      w->second.labeled_classification = cll;
    }
  } catch (...) {
    std::cout << "oi check yo self" << std::endl;
  }

  marker_manager_.update_markers(id_object_map_);
  ogrid_manager_.update_ogrid(id_object_map_, latest_odom_);

  pub_objects_.publish(objects);
}

bool pcodar_controller::bounds_update_cb(const mil_bounds::BoundsConfig &config) {
  ROS_INFO("Updating bounds...");

  geometry_msgs::TransformStamped transform;
  try {
    transform = tf_buffer_.lookupTransform(
        "enu", config.frame,
        ros::Time::now(),
        ros::Duration(1, 0));  // change time to pcloud header? pcloud->header.stamp
  } catch (tf2::TransformException &ex) {
    ROS_ERROR("%s", ex.what());
    return false;
  }

  boundary_t untransformed_boundary;

  untransformed_boundary[0](0) = config.x1;
  untransformed_boundary[0](1) = config.y1;
  untransformed_boundary[0](2) = config.z1;
  untransformed_boundary[1](0) = config.x2;
  untransformed_boundary[1](1) = config.y2;
  untransformed_boundary[1](2) = config.z2;
  untransformed_boundary[2](0) = config.x3;
  untransformed_boundary[2](1) = config.y3;
  untransformed_boundary[2](2) = config.z3;
  untransformed_boundary[3](0) = config.x4;
  untransformed_boundary[3](1) = config.y4;
  untransformed_boundary[3](2) = config.z4;

  tf2::doTransform(untransformed_boundary[0], boundary[0], transform);
  tf2::doTransform(untransformed_boundary[1], boundary[1], transform);
  tf2::doTransform(untransformed_boundary[2], boundary[2], transform);
  tf2::doTransform(untransformed_boundary[3], boundary[3], transform);
  ROS_INFO("bounds updateded");
}

bool pcodar_controller::DBQuery_cb(mil_msgs::ObjectDBQuery::Request &req,
                                  mil_msgs::ObjectDBQuery::Response &res) {
  if (req.cmd != "") {
    int pos = req.cmd.find_first_of("=");

    int id = -1;
    try {
      id = std::stoi(req.cmd.substr(0, pos));
    } catch (...) {
      std::cout << "Could not find id " << id << std::endl;
      res.found = false;
      return false;
    }
    std::string cmd = req.cmd.substr(pos + 1);
    auto it = id_object_map_->find(id);
    if (it == id_object_map_->end()) {
      std::cout << "Could not find id " << id << std::endl;
      return false;
    } else {
      std::cout << "set " << id << " to " << cmd << std::endl;
      it->second.labeled_classification = cmd;
    }
  }
  std::vector<mil_msgs::PerceptionObject> objects(id_object_map_->size());
  int i = 0;
  for (auto &o : *id_object_map_) {
    objects[i] = o.second;
    i++;
  }
  if (req.name == "all") {
    res.found = true;
    res.objects = objects;
    return true;
  }

  // if (std::find(classification_strings.begin(), classification_strings.end(),
                // req.name) != classification_strings.end()) {
  if (req.name != "")
    for (const auto &object : objects) {
      if (object.classification == req.name ||
          object.labeled_classification == req.name) {
        res.found = true;
        res.objects.push_back(object);
      }
    }
  // }
  return true;
}

}  // namespace pcodar
