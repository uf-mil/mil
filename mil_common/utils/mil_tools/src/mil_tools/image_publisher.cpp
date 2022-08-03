#include <mil_tools/image_publisher.hpp>

ImagePublisher::ImagePublisher(const std::string& topic, const std::string& encoding, int queue_size) : it_(nh_)
{
  this->encoding_ = encoding;
  pub_ = it_.advertise(topic, queue_size);
}
void ImagePublisher::publish(cv::Mat& image_arg)
{
  std_msgs::Header header;
  header.stamp = ros::Time::now();
  sensor_msgs::ImagePtr msg = cv_bridge::CvImage(header, encoding_, image_arg).toImageMsg();
  pub_.publish(msg);
}
