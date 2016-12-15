#include "CameraLidarTransformer.hpp"

ros::Duration CameraLidarTransformer::MAX_TIME_ERR = ros::Duration(0,6E7);
CameraLidarTransformer::CameraLidarTransformer()
    : nh(ros::this_node::getName()),
      tfBuffer(),
      tfListener(tfBuffer, nh),
      lidarSub(nh, "/velodyne_points", 10),
      lidarCache(lidarSub, 10),
      camera_info_received(false)
      #ifdef DO_ROS_DEBUG
      ,
      image_transport(nh)
      #endif
{
  #ifdef DO_ROS_DEBUG
  points_debug_publisher = image_transport.advertise("points_debug", 1);
  pubMarkers = nh.advertise<visualization_msgs::MarkerArray>("markers_debug", 10);
  #endif
  nh.param<std::string>("camera_info_topic",camera_info_topic,"/right/right/camera_info");
  cameraInfoSub = nh.subscribe(camera_info_topic, 1, &CameraLidarTransformer::cameraInfoCallback, this);
  std::string camera_to_lidar_transform_topic;
  nh.param<std::string>("camera_to_lidar_transform_topic",camera_to_lidar_transform_topic,"transform_camera");
  transformServiceServer = nh.advertiseService(camera_to_lidar_transform_topic, &CameraLidarTransformer::transformServiceCallback, this);
}
void CameraLidarTransformer::cameraInfoCallback(sensor_msgs::CameraInfo info)
{
  camera_info = info;
  cam_model.fromCameraInfo(camera_info);
  camera_info_received = true;
}
bool CameraLidarTransformer::inCameraFrame(cv::Point2d& point)
{
  return point.x > 0 && point.x < camera_info.width &&
         point.y > 0 && point.y < camera_info.height;
}
void CameraLidarTransformer::drawPoint(cv::Mat& mat, cv::Point2d& point, cv::Scalar color)
{
  if (point.x - 20 > 0 && point.x + 20 < mat.cols && point.y - 20 > 0 && point.y + 20 < mat.rows)
    cv::circle(mat, point, 3, color, -1);
}
bool CameraLidarTransformer::transformServiceCallback(navigator_msgs::CameraToLidarTransform::Request &req, navigator_msgs::CameraToLidarTransform::Response &res)
{
  if (!camera_info_received)
  {
    res.success = false;
    res.error = "NO CAMERA INFO";
    return true;
  }
  if (camera_info.header.frame_id != req.header.frame_id)
  {
    res.success = false;
    res.error = "DIFFERENT FRAME ID THAN SUBSCRIBED CAMERA";
    return true;
  }
  visualization_msgs::MarkerArray markers;
  std::vector<sensor_msgs::PointCloud2ConstPtr> nearbyLidar = lidarCache.getInterval(req.header.stamp-MAX_TIME_ERR, req.header.stamp+MAX_TIME_ERR);
  sensor_msgs::PointCloud2ConstPtr scloud;
  ros::Duration minErr = ros::Duration(5000,0);
  for (auto& pc : nearbyLidar)
  {
    ros::Duration thisErr = pc->header.stamp - req.header.stamp;
    if (thisErr < minErr)
    {
      scloud = pc;
      minErr = thisErr;
    }
  }
  if (!scloud) {
    res.success = false;
    res.error =  navigator_msgs::CameraToLidarTransform::Response::CLOUD_NOT_FOUND;
    return true;
  }
  if (!tfBuffer.canTransform(req.header.frame_id, "velodyne", ros::Time(0), ros::Duration(1)))
  {
    res.success = false;
    res.error = "NO TRANSFORM";
    return true;
  }
  geometry_msgs::TransformStamped transform = tfBuffer.lookupTransform(req.header.frame_id, "velodyne", ros::Time(0));
  sensor_msgs::PointCloud2 cloud_transformed;
  tf2::doTransform(*scloud, cloud_transformed, transform);

  #ifdef DO_ROS_DEBUG
  cv::Mat debug_image(camera_info.height, camera_info.width, CV_8UC3, cv::Scalar(0));
  cv::circle(debug_image, cv::Point(req.point.x, req.point.y), 8, cv::Scalar(255, 0, 0), -1);
  #endif

  double minDistance = std::numeric_limits<double>::max();
  pcl::PointCloud<pcl::PointXYZ> cloud;
  pcl::fromROSMsg(cloud_transformed, cloud);
  std::vector<int> indices;
  for (unsigned index = 0; index < cloud.size(); index++)
  {
      pcl::PointXYZ& p = cloud[index];
      if (p.z < 0 || p.z > 30) continue;
      cv::Point2d point = cam_model.project3dToPixel(cv::Point3d(p.x, p.y, p.z));
      if (inCameraFrame(point))
      {
          #ifdef DO_ROS_DEBUG
          visualization_msgs::Marker marker_point;
          marker_point.header = req.header;
          marker_point.header.seq = 0;
          marker_point.header.frame_id = req.header.frame_id;
          marker_point.id = index;
          marker_point.type = visualization_msgs::Marker::CUBE;
          marker_point.pose.position.x = p.x;
          marker_point.pose.position.y = p.y;
          marker_point.pose.position.z = p.z;
          marker_point.scale.x = 0.1;
          marker_point.scale.y = 0.1;
          marker_point.scale.z = 0.1;
          marker_point.color.a = 1.0;
          marker_point.color.r = 0.0;
          marker_point.color.g = 1.0;
          marker_point.color.b = 0.0;
          drawPoint(debug_image, point);
          #endif
          double distance = sqrt(pow(point.x - req.point.x ,2) + pow(point.y - req.point.y,2) ); //Distance (2D) from request point to projected lidar point
          if (distance < req.tolerance)
          {
              geometry_msgs::Point geo_point;
              geo_point.x = p.x;
              geo_point.y = p.y;
              geo_point.z = p.z;
              if (distance < minDistance)
              {
                res.closest = geo_point;
                minDistance = distance;
              }
              indices.push_back(index);

              res.transformed.push_back(geo_point);

              #ifdef DO_ROS_DEBUG
              drawPoint(debug_image, point, cv::Scalar(0, 255, 0));
              marker_point.color.r = 1.0;
              marker_point.color.g = 0.0;
              marker_point.color.b = 0.0;
              #endif
          }
          #ifdef DO_ROS_DEBUG
          markers.markers.push_back(marker_point);
          #endif 

      }
  }
  if (res.transformed.size() > 0) {
    float x,y,z,n;
    pcl::NormalEstimation<pcl::PointXYZ,pcl::Normal > ne;
    ne.computePointNormal (cloud, indices, x, y, z, n);
    Eigen::Vector3d normal_vector = Eigen::Vector3d(x,y,z).normalized();
    res.normal.x = -normal_vector(0, 0);
    res.normal.y = -normal_vector(1, 0);
    res.normal.z = -normal_vector(2, 0);

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
    #endif

    res.distance = res.closest.z;
    res.success = true;
  } else {
    res.error = navigator_msgs::CameraToLidarTransform::Response::NO_POINTS_FOUND;
    res.success = false;
  }

#ifdef DO_ROS_DEBUG
  //Publish 3D debug market
  pubMarkers.publish(markers);
  //Publish debug image
  cv_bridge::CvImage ros_debug_image;
  ros_debug_image.encoding = "bgr8";
  ros_debug_image.image = debug_image.clone();
  points_debug_publisher.publish(ros_debug_image.toImageMsg());
#endif

  return true;
}
