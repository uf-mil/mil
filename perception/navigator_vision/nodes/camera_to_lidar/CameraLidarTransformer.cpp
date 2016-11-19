#include "CameraLidarTransformer.hpp"

CameraLidarTransformer::CameraLidarTransformer()
    : nh(ros::this_node::getName()),
      tfBuffer(),
      tfListener(tfBuffer, nh),
      lidarSub(nh, "/velodyne_points", 10),
      lidarCache(lidarSub, 10)
#ifdef DO_ROS_DEBUG
      ,
      image_transport(nh)
#endif
{
#ifdef DO_ROS_DEBUG
  points_debug_publisher =
      image_transport.advertise("points_debug", 1);
  pubMarkers = nh.advertise<visualization_msgs::MarkerArray>(
      "markers_debug", 10);
#endif
  nh.param<std::string>("camera_info_topic",camera_info_topic,"/right/right/camera_info");
  printf("Topic %s\n",camera_info_topic.c_str());
  // ~nh.param<double>("MIN_Z",MIN_Z,1);
  // ~nh.param<double>("MAX_Z_ERROR",MAX_Z_ERROR,0.2);
  std::cout << "Constructed" << std::endl;
  cameraInfoSub =
      nh.subscribe(camera_info_topic, 1,
                   &CameraLidarTransformer::cameraInfoCallback, this);
  std::string camera_to_lidar_transform_topic;
  nh.param<std::string>("camera_to_lidar_transform_topic",camera_to_lidar_transform_topic,"transform_camera");
  transformServiceServer = nh.advertiseService(
      camera_to_lidar_transform_topic, &CameraLidarTransformer::transformServiceCallback,
      this);
}
void CameraLidarTransformer::cameraInfoCallback(sensor_msgs::CameraInfo info)
{
  camera_info = info;
}
bool CameraLidarTransformer::transformServiceCallback(
    navigator_msgs::CameraToLidarTransform::Request &req,
    navigator_msgs::CameraToLidarTransform::Response &res) {
  visualization_msgs::MarkerArray markers;
  sensor_msgs::PointCloud2ConstPtr scloud =
      lidarCache.getElemAfterTime(req.header.stamp);
  if (!scloud) {
    res.success = false;
    res.error =  navigator_msgs::CameraToLidarTransform::Response::CLOUD_NOT_FOUND;
    return true;
  }
  geometry_msgs::TransformStamped transform =
      tfBuffer.lookupTransform(req.header.frame_id, "velodyne", req.header.stamp);
  sensor_msgs::PointCloud2 cloud_transformed;
  tf2::doTransform(*scloud, cloud_transformed, transform);
#ifdef DO_ROS_DEBUG
  cv::Mat debug_image(camera_info.height, camera_info.width, CV_8UC3,
                      cv::Scalar(0));
#endif
  cv::circle(debug_image, cv::Point(req.point.x, req.point.y), 8,
             cv::Scalar(255, 0, 0), -1);
  //Tracks the closest lidar point to the requested camera point
  double minDistance = std::numeric_limits<double>::max();
  pcl::PointCloud<pcl::PointXYZ> cloud2;
  pcl::fromROSMsg(cloud_transformed, cloud2);
  std::vector<int> indices;
  for (auto ii = 0, jj = 0; ii < cloud_transformed.width;
       ++ii, jj += cloud_transformed.point_step) {
    floatConverter x, y, z, i;
    for (int kk = 0; kk < 4; ++kk) {
      x.data[kk] = cloud_transformed.data[jj + kk];
      y.data[kk] = cloud_transformed.data[jj + 4 + kk];
      z.data[kk] = cloud_transformed.data[jj + 8 + kk];
      i.data[kk] = cloud_transformed.data[jj + 16 + kk];
    }
    cam_model.fromCameraInfo(camera_info);
    if (int(z.f) < 0 || int(z.f) > 30) continue;
    cv::Point2d point = cam_model.project3dToPixel(cv::Point3d(x.f, y.f, z.f));

    if (int(point.x) > 0 && int(point.x) < camera_info.width &&
        int(point.y) > 0 && int(point.y) < camera_info.height) {
#ifdef DO_ROS_DEBUG
      visualization_msgs::Marker marker_point;
      marker_point.header = req.header;
      marker_point.header.seq = 0;
      marker_point.header.frame_id = req.header.frame_id;
      marker_point.id = (int)ii;
      marker_point.type = visualization_msgs::Marker::CUBE;
      marker_point.pose.position.x = x.f;
      marker_point.pose.position.y = y.f;
      marker_point.pose.position.z = z.f;
      marker_point.scale.x = 0.1;
      marker_point.scale.y = 0.1;
      marker_point.scale.z = 0.1;
      marker_point.color.a = 1.0;
      marker_point.color.r = 0.0;
      marker_point.color.g = 1.0;
      marker_point.color.b = 0.0;
      //marker_point.lifetime = ros::Duration(0.5);

      // Draw
      if (int(point.x - 20) > 0 && int(point.x + 20) < camera_info.width &&
          int(point.y - 20) > 0 && int(point.y + 20) < camera_info.height) {
        cv::circle(debug_image, point, 3,
                   cv::Scalar(0, 0, 255 * std::min(1.0, z.f / 20.0)), -1);
      }
#endif
      double distance = sqrt(pow(point.x - req.point.x ,2) + pow(point.y - req.point.y,2) ); //Distance (2D) from request point to projected lidar point
      if (distance < req.tolerance) {
#ifdef DO_ROS_DEBUG
        if (int(point.x - 20) > 0 && int(point.x + 20) < camera_info.width &&
            int(point.y - 20) > 0 && int(point.y + 20) < camera_info.height) {
          cv::circle(debug_image, point, 3, cv::Scalar(0, 255, 0), -1);
        }
#endif

        geometry_msgs::Point geo_point;
        geo_point.x = x.f;
        geo_point.y = y.f;
        geo_point.z = z.f;
        if (distance < minDistance)
        {
          res.closest = geo_point;
          minDistance = distance;
        }
        indices.push_back(ii);
        marker_point.color.r = 1.0;
        marker_point.color.g = 0.0;
        marker_point.color.b = 0.0;
        res.transformed.push_back(geo_point);
      }
      markers.markers.push_back(marker_point);  

    }
  }
  if (res.transformed.size() < 1) {
    res.success = false;
    res.error = navigator_msgs::CameraToLidarTransform::Response::NO_POINTS_FOUND;
    return true;
  }

  float x,y,z,n;
  pcl::NormalEstimation<pcl::PointXYZ,pcl::Normal > ne;
  ne.computePointNormal (cloud2,indices,x,y,z,n);
  Eigen::Vector3d normal_vector = Eigen::Vector3d(x,y,z).normalized();
  res.normal.x = -normal_vector(0, 0);
  res.normal.y = -normal_vector(1, 0);
  res.normal.z = -normal_vector(2, 0);

  res.distance = res.closest.z;

#ifdef DO_ROS_DEBUG
  //Add a marker for the normal to the plane
  geometry_msgs::Point sdp_normalvec_ros;
  sdp_normalvec_ros.x = res.closest.x + res.normal.x;
  sdp_normalvec_ros.y = res.closest.y + res.normal.y;
  sdp_normalvec_ros.z = res.closest.z + res.normal.z;
  visualization_msgs::Marker marker_normal;
  marker_normal.header = req.header;
  marker_normal.header.seq = 0;
  marker_normal.header.frame_id =  req.header.frame_id;
  marker_normal.id = 3000;
  marker_normal.type = visualization_msgs::Marker::ARROW;
  marker_normal.points.push_back(res.closest);
  marker_normal.points.push_back(sdp_normalvec_ros);
  marker_normal.scale.x = 0.1;
  marker_normal.scale.y = 0.5;
  marker_normal.scale.z = 0.5;
  marker_normal.color.a = 1.0;
  marker_normal.color.r = 0.0;
  marker_normal.color.g = 0.0;
  marker_normal.color.b = 1.0;
  markers.markers.push_back(marker_normal);
  
  //Publish 3D debug market
  pubMarkers.publish(markers);
  
  //Publish debug image
  cv_bridge::CvImage ros_debug_image;
  ros_debug_image.encoding = "bgr8";
  ros_debug_image.image = debug_image.clone();
  points_debug_publisher.publish(ros_debug_image.toImageMsg());
#endif
  res.success = true;
  return true;
}
