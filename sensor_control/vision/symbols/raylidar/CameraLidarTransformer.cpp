#include "CameraLidarTransformer.hpp"

union floatConverter
{
    float f; //This is... brilliant.
    struct
    {
        uint8_t data[4];
    };
};

CameraLidarTransformer::CameraLidarTransformer()
    : nh("camera_lidar_transformer"),
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
      image_transport.advertise("/camera_lidar_transformer/points_debug", 1);
  debugCloudPub = nh.advertise<sensor_msgs::PointCloud2>(
      "/camera_lidar_transformer/cloud_debug", 20);
  pubMarkers = nh.advertise<visualization_msgs::MarkerArray>(
      "/dock_shapes/raylider/markers", 10);
#endif
  std::cout << "Constructed" << std::endl;
  cameraInfoSub =
      nh.subscribe("/right/right/camera_info", 1,
                   &CameraLidarTransformer::cameraInfoCallback, this);
  transformServiceServer = nh.advertiseService(
      "transform_camera", &CameraLidarTransformer::transformServiceCallback,
      this);
}
void CameraLidarTransformer::cameraInfoCallback(sensor_msgs::CameraInfo info)
{
  // ~printf("Camera Info: Width=%d height=%d\n",info.width,info.height);
  camera_info = info;
}
bool CameraLidarTransformer::transformServiceCallback(
    navigator_msgs::CameraToLidarTransform::Request &req,
    navigator_msgs::CameraToLidarTransform::Response &res) {
  visualization_msgs::MarkerArray markers;
  sensor_msgs::PointCloud2ConstPtr scloud =
      lidarCache.getElemAfterTime(req.header.stamp);
  if (!scloud) {
    std::cout << "Cloud not found" << std::endl;
    res.success = false;
    res.error = "CLOUD NOT FOUND";
    return true;
  }
  geometry_msgs::TransformStamped transform =
      tfBuffer.lookupTransform("right_right_cam", "velodyne", req.header.stamp);
  sensor_msgs::PointCloud2 cloud_transformed;
  tf2::doTransform(*scloud, cloud_transformed, transform);
// ~geometry_msgs::Vector3Stamped x;
// ~geometry_msgs::Vector3Stamped y;
// ~tf2::doTransform (x,y,transform);
#ifdef DO_ROS_DEBUG
  debugCloudPub.publish(cloud_transformed);
  cv::Mat debug_image(camera_info.height, camera_info.width, CV_8UC3,
                      cv::Scalar(0));
#endif
  int found = 0;
  using namespace boost::accumulators;
  accumulator_set<double, features<tag::mean, tag::variance>> zAcc;

  Eigen::Matrix3d matrixFindPlaneA;
  matrixFindPlaneA << 0, 0, 0, 0, 0, 0, 0, 0, 0;
  Eigen::Vector3d matrixFindPlaneB(0, 0, 0);

  cv::circle(debug_image, cv::Point(req.point.x, req.point.y), 3,
             cv::Scalar(255, 0, 0), 5);
  double minDistance = 100000;
  for (auto ii = 0, jj = 0; ii < cloud_transformed.width;
       ++ii, jj += cloud_transformed.point_step) {
    floatConverter x, y, z, i;
    for (int kk = 0; kk < 4; ++kk) {
      x.data[kk] = cloud_transformed.data[jj + kk];
      y.data[kk] = cloud_transformed.data[jj + 4 + kk];
      z.data[kk] = cloud_transformed.data[jj + 8 + kk];
      i.data[kk] = cloud_transformed.data[jj + 16 + kk];
    }
    //camera_info.K[0] = 314.074175;
    //camera_info.K[2] = 324.843873;
    //camera_info.K[4] = 314.074175;
    //camera_info.K[5] = 233.077192;
    //camera_info.K[8] = 1;
    //camera_info.D[0] = -0.23943105096248707;
    //camera_info.D[1] = 0.047913831179275175;
    //camera_info.D[2] = -0.0010221177506805387;
    //camera_info.D[3] = 0.0007312338670705555;
    //camera_info.R[0] = 1.0;
    //camera_info.R[4] = 1.0;
    //camera_info.R[8] = 1.0;
    //camera_info.P[0] = 226.95376586914062;
    //camera_info.P[2] = 324.88712003802357;
    //camera_info.P[5] = 259.2882690429687;
    //camera_info.P[6] = 227.53713928460274;
    cam_model.fromCameraInfo(camera_info);
    if (int(z.f) < 0 || int(z.f) > 30) continue;
    cv::Point2d point = cam_model.project3dToPixel(cv::Point3d(x.f, y.f, z.f));

    if (int(point.x) > 0 && int(point.x) < camera_info.width &&
        int(point.y) > 0 && int(point.y) < camera_info.height) {
// ~printf("Point: x=%i y=%i WIDTH=%i
// HEIGHT=%i\n",int(point.x),int(point.y),camera_info.width,camera_info.height);
#ifdef DO_ROS_DEBUG
      visualization_msgs::Marker marker_point;
      marker_point.header = req.header;
      marker_point.header.seq = 0;
      marker_point.header.frame_id = "right_right_cam";
      marker_point.id = (int)ii;
      marker_point.type = visualization_msgs::Marker::CUBE;
      marker_point.pose.position.x = x.f;
      marker_point.pose.position.y = y.f;
      marker_point.pose.position.z = z.f;
      marker_point.scale.x = 1;
      marker_point.scale.y = 0.1;
      marker_point.scale.z = 0.1;
      marker_point.color.a = 1.0;
      marker_point.color.r = 0.0;
      marker_point.color.g = 1.0;
      marker_point.color.b = 0.0;
      marker_point.lifetime = ros::Duration(0.5);
      markers.markers.push_back(marker_point);

      // Draw
      if (int(point.x - 20) > 0 && int(point.x + 20) < camera_info.width &&
          int(point.y - 20) > 0 && int(point.y + 20) < camera_info.height) {
        cv::circle(debug_image, point, 3,
                   cv::Scalar(0, 0, 255 * std::min(1.0, z.f / 20.0)), -1);
      }
#endif
      geometry_msgs::Point geo_point;
      double distance = sqrt(pow(point.x - req.point.x ,2) + pow(point.y - req.point.y,2) );
      if (abs(int(point.x) - req.point.x) < req.tolerance &&
          abs(int(point.y) - req.point.y) < req.tolerance) {
#ifdef DO_ROS_DEBUG
        if (int(point.x - 20) > 0 && int(point.x + 20) < camera_info.width &&
            int(point.y - 20) > 0 && int(point.y + 20) < camera_info.height) {
          cv::circle(debug_image, point, 3, cv::Scalar(0, 255, 0), -1);
        }
#endif
        matrixFindPlaneA(0, 0) += x.f * x.f;
        matrixFindPlaneA(1, 0) += y.f * x.f;
        matrixFindPlaneA(2, 0) += x.f;
        matrixFindPlaneA(0, 1) += x.f * y.f;
        matrixFindPlaneA(1, 1) += y.f * y.f;
        matrixFindPlaneA(2, 1) += y.f;
        matrixFindPlaneA(0, 2) += x.f;
        matrixFindPlaneA(1, 2) += y.f;
        matrixFindPlaneA(2, 2) += 1;
        matrixFindPlaneB(0, 0) -= x.f * z.f;
        matrixFindPlaneB(1, 0) -= y.f * z.f;
        matrixFindPlaneB(2, 0) -= z.f;
        zAcc(z.f);
        geo_point.x = x.f;
        geo_point.y = y.f;
        geo_point.z = z.f;
        if (distance < minDistance)
        {
          res.closest = geo_point;
          minDistance = distance;
        }
        res.transformed.push_back(geo_point);
      }
    }
  }
  if (res.transformed.size() < 1) {
    res.success = false;
    res.error = "NO POINTS FOUND";
    return true;
  }

  res.distance = boost::accumulators::mean(zAcc);
  Eigen::Vector3d normalvec = matrixFindPlaneA.colPivHouseholderQr()
                                  .solve(matrixFindPlaneB)
                                  .normalized();
  geometry_msgs::Vector3 normalvec_ros;
  normalvec_ros.x = normalvec(0, 0);
  normalvec_ros.y = normalvec(1, 0);
  normalvec_ros.z = normalvec(2, 0);
  res.normal = normalvec_ros;
  std::cout << "Plane solution: " << normalvec << std::endl;

#ifdef DO_ROS_DEBUG
  //////////////////VISUALIZATION FOR NORMAL////////////////////////
  visualization_msgs::Marker marker_normal;
  marker_normal.header = req.header;
  marker_normal.header.seq = 0;
  marker_normal.header.frame_id = "right_right_cam";
  marker_normal.id = 3000;
  marker_normal.type = visualization_msgs::Marker::ARROW;

  geometry_msgs::Point sdp_normalvec_ros;
  sdp_normalvec_ros.x = normalvec_ros.x + res.transformed[0].x;
  sdp_normalvec_ros.y = normalvec_ros.y + res.transformed[0].y;
  sdp_normalvec_ros.z = normalvec_ros.z + res.transformed[0].z;
  marker_normal.points.push_back(res.transformed[0]);
  marker_normal.points.push_back(sdp_normalvec_ros);

  marker_normal.scale.x = 0.1;
  marker_normal.scale.y = 0.5;
  marker_normal.scale.z = 0.5;
  marker_normal.color.a = 1.0;
  marker_normal.color.r = 1.0;
  marker_normal.color.g = 0.0;
  marker_normal.color.b = 0.0;

  markers.markers.push_back(marker_normal);
  cv_bridge::CvImage ros_debug_image;
  ros_debug_image.encoding = "bgr8";
  ;
  ros_debug_image.image = debug_image.clone();
  points_debug_publisher.publish(ros_debug_image.toImageMsg());
  pubMarkers.publish(markers);
#endif
  res.success = true;
  return true;
}
