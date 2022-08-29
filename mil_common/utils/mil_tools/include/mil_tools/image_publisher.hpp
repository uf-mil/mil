#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
#include <ros/ros.h>

#include <opencv2/highgui/highgui.hpp>

/**
 * Abstraction for publishing images to without the necessary boiler plate.
 * All that's needed to publish an image is a cv::Mat.
 */
class ImagePublisher
{
public:
  /**
   * Creates an ImagePublisher in the namespace of nh.
   * @param nh NodeHandle to have node in the correct namespace.
   * @param topic Camera topic to publish images to.
   * @param encoding Type of image encoding
   * @param queue_size Size of the queue for the publisher.
   */
  ImagePublisher(const ros::NodeHandle& nh, const std::string& topic, const std::string& encoding = "bgr8",
                 int queue_size = 1);

  /**
   * Publishes an OpenCV image.
   * @param image_arg OpenCV image to publish.
   */
  void publish(cv::Mat& image_arg);

private:
  std::string encoding_;
  image_transport::Publisher pub_;
  image_transport::ImageTransport it_;
};
