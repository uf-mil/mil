#include <point_cloud_object_detection_and_recognition/ogrid_manager.hpp>

namespace pcodar
{
void ogrid_manager::initialize(ros::NodeHandle& nh) {
  pub_ogrid_ = nh.advertise<nav_msgs::OccupancyGrid>("ogrid", 5);
  ogrid_mat_ = cv::Mat(10000*0.3, 10000*0.3, CV_8UC1, cv::Scalar(0));
  ogrid_.info.resolution = .3;
  ogrid_.header.frame_id = "enu";
  ogrid_.info.width = 10000*0.3;
  ogrid_.info.height = 10000*0.3;
}
void ogrid_manager::update_ogrid(const mil_msgs::PerceptionObjectArray &objects, nav_msgs::OdometryConstPtr odom)
{
  if (objects.objects.empty()) return;
  ogrid_mat_ = cv::Scalar(0);
  for (const auto& object : objects.objects)
  {
    std::cout << "Pose: " << object.pose.position.x << " " << object.pose.position.y << " " << object.pose.position.z << std::endl;

  // auto x = object.pose.position.x + 201/2 - odom->pose.pose.position.x;
  // auto y = object.pose.position.y + 201/2 - odom->pose.pose.position.y;
  // auto z = object.pose.position.z + 201/2 - odom->pose.pose.position.z;
  
  auto x = object.pose.position.x/0.3 + 10000*0.3/2;
  auto y = object.pose.position.y/0.3 + 10000*0.3/2;
  auto z = object.pose.position.z/0.3 + 10000*0.3/2;
  std::cout << x << " " << y << " " << z << std::endl;

    cv::circle(ogrid_mat_, cv::Point(x, y), 10, 255, -1);
  }
  
    std::vector<int8_t> data(ogrid_mat_.cols * ogrid_mat_.rows);
      auto out_it = data.begin();
        for (int row = 0; row < ogrid_mat_.rows; ++row)
            {
                  auto *p = ogrid_mat_.ptr(row);
                      for (int col = 0; col < ogrid_mat_.cols; ++col)
                            {
                                    *out_it = int(*p++);
                                          out_it++;
                                              }
            }
  ogrid_.header.stamp = ros::Time::now();
  ogrid_.info.origin.position.x = -10000*0.09/2;
  ogrid_.info.origin.position.y = -10000*0.09/2;
  ogrid_.info.origin.orientation.w = 1;
  ogrid_.data = data;
  // cv::imshow("test", ogrid_mat_);
  // cv::waitKey(30);
  pub_ogrid_.publish(ogrid_);
}
}
