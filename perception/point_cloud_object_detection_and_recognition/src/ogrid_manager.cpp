#include <point_cloud_object_detection_and_recognition/ogrid_manager.hpp>

namespace pcodar
{
void ogrid_manager::initialize(ros::NodeHandle& nh) {
  pub_ogrid_ = nh.advertise<nav_msgs::OccupancyGrid>("/ogrid", 5);
  ogrid_mat_ = cv::Mat(10000*0.3, 10000*0.3, CV_8UC1, cv::Scalar(0));
  ogrid_.info.resolution = .3;
  ogrid_.header.frame_id = "enu";
  ogrid_.info.width = 10000*0.3;
  ogrid_.info.height = 10000*0.3;
}
void ogrid_manager::draw_boundary() {
  std::vector<cv::Point>bounds(pcodar::boundary.size());
  for(int i = 0; i < bounds.size(); ++i) {
    bounds[i].x = boundary[i](0)/0.3 + 10000*0.3/2;
    bounds[i].y = boundary[i](1)/0.3 + 10000*0.3/2;
  }


  for(int i = 0; i < bounds.size(); ++i) {
    // std::cout << bounds[i] << std::endl;
    cv::circle(ogrid_mat_, bounds[i], 15, cv::Scalar(99), -1);
  }
  const cv::Point *pts = (const cv::Point*) cv::Mat(bounds).data;
  int npts = cv::Mat(bounds).rows;

  // std::cout <<  "Number of polygon vertices: " <<  npts <<  std::endl;
   // draw the polygon 
  cv::polylines(ogrid_mat_, &pts, &npts, 1, true, cv::Scalar(99), 5);

  // Eigen::Vector2d(-210, -175), Eigen::Vector2d(-210, 15), Eigen::Vector2d(-50, 15), Eigen::Vector2d(-50, -175)
}
void ogrid_manager::update_ogrid(const id_object_map_ptr objects, nav_msgs::OdometryConstPtr odom)
{
  if (objects->empty()) return;
  ogrid_mat_ = cv::Scalar(0);
  for (const auto& object : *objects)
  {
//std::cout << "Pose: " << object.pose.position.x << " " << object.pose.position.y << " " << object.pose.position.z << std::endl;

  // auto x = object.pose.position.x + 201/2 - odom->pose.pose.position.x;
  // auto y = object.pose.position.y + 201/2 - odom->pose.pose.position.y;
  // auto z = object.pose.position.z + 201/2 - odom->pose.pose.position.z;
  
  auto x = object.second.pose.position.x/0.3 + 10000*0.3/2;
  auto y = object.second.pose.position.y/0.3 + 10000*0.3/2;
  auto z = object.second.pose.position.z/0.3 + 10000*0.3/2;
  //std::cout << object.pose.orientation.x << " " << object.pose.orientation.y << " " << object.pose.orientation.z << " " << object.pose.orientation.w << std::endl;
  //std::cout << x << " " << y << " " << z << std::endl;
    //double angle = 2 * acos(object.pose.orientation.w);
  // std::cout << object.second.points.size() << std::endl;
  for(const auto &point : object.second.points) {

    cv::circle(ogrid_mat_, cv::Point(point.x/0.3 + 10000*0.3/2, point.y/0.3 + 10000*0.3/2), params.ogrid_inflation_cell, cv::Scalar(99), -1);
  }
    // auto q = object.second.pose.orientation;
    // double siny = 2.0 * (q.w * q.z);
    // double cosy = 1.0 - 2.0 * (q.z*q.z);
    // double angle = atan2(siny, cosy);
    // cv::RotatedRect rRect(cv::Point(x,y), cv::Size(object.second.scale.x/0.3, object.second.scale.y/0.3), angle * 180 / 3.1415);
    // cv::Point2f vertices[4];
    // rRect.points(vertices);
    // cv::Point vert[4];
    // for (int i = 0; i < 4; i++)
    //     vert[i] = vertices[i];
    // cv::fillConvexPoly(ogrid_mat_, vert, 4, cv::Scalar(99));
  }
  draw_boundary();
  
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
