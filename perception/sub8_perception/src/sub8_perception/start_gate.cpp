#include <sub8_perception/start_gate.hpp>
Sub8StartGateDetector::Sub8StartGateDetector() : running_(true), image_transport_(nh_), gate_line_buffer_(30)
{
  image_sub_ = image_transport_.subscribeCamera("/camera/front/right/image_rect_color", 1,
                                                &Sub8StartGateDetector::imageCallback, this);
  service_2d_ =
      nh_.advertiseService("/vision/start_gate/pose", &Sub8StartGateDetector::requestStartGatePosition2d, this);
  service_enable_ =
      nh_.advertiseService("/vision/start_gate/enable", &Sub8StartGateDetector::requestStartGateEnable, this);
  service_distance_ =
      nh_.advertiseService("/vision/start_gate/distance", &Sub8StartGateDetector::requestStartGateDistance, this);
  pubImage_ = image_transport_.advertise("/start_gate/debug_image", 1);
}
Sub8StartGateDetector::~Sub8StartGateDetector()
{
}

void Sub8StartGateDetector::imageCallback(const sensor_msgs::ImageConstPtr &image_msg,
                                          const sensor_msgs::CameraInfoConstPtr &info_msg)
{
  if (running_)
  {
    // Get some image/camera info
    cam_model_.fromCameraInfo(info_msg);
    image_time_ = image_msg->header.stamp;

    // If we have a large enough sample of gate identifications:
    if (gate_line_buffer_.size() > gate_line_buffer_.capacity() - 1)
    {
      // Loop through the buffer and save mean and deviations of both size and
      // position
      for (auto &gate1 : gate_line_buffer_)
      {
        for (auto &gate2 : gate1)
        {
          cv::Point2i s = gate2[1] - gate2[0];
          cv::Point2i center = cv::Point2i(s.x / 2 + s.x, s.y / 2 + s.y);
          accX_(center.x);
          accY_(center.y);
          accSizeX_(s.x);
          accSizeY_(s.y);
        }
      }

      // Look at one of the elements (gate_line_buffer_.capacity()-1) in the
      // circular buffer and check it's statistics and delete if necessary
      for (size_t i = 0; i < gate_line_buffer_[gate_line_buffer_.capacity() - 1].size(); i++)
      {
        cv::Point2i s = gate_line_buffer_[gate_line_buffer_.capacity() - 1][i][1] -
                        gate_line_buffer_[gate_line_buffer_.capacity() - 1][i][0];
        cv::Point2i center = cv::Point2i(s.x / 2 + s.x, s.y / 2 + s.y);

        // If position of the element is greater than one deviation from the
        // mean, then delete
        if (abs(center.x - mean(accX_)) > sqrt(variance(accX_)) || abs(center.y - mean(accY_) > sqrt(variance(accY_))))
        {
          gate_line_buffer_[gate_line_buffer_.capacity() - 1].erase(
              gate_line_buffer_[gate_line_buffer_.capacity() - 1].begin() + i);
          ROS_INFO_STREAM("Deleted center: " << center);
          i--;
          continue;
        }

        // If the size of the element is greater than one deviation from the
        // mean, then delete
        if (abs(s.x - mean(accSizeX_)) > sqrt(variance(accSizeX_)) ||
            abs(s.y - mean(accSizeY_)) > sqrt(variance(accSizeY_)))
        {
          gate_line_buffer_[gate_line_buffer_.capacity() - 1].erase(
              gate_line_buffer_[gate_line_buffer_.capacity() - 1].begin() + i);
          ROS_INFO_STREAM("Deleted size: " << s);
          i--;
          continue;
        }
      }

      // Visualize one of the saved gates in the buffer
      cv::Mat show = cv_bridge::toCvShare(image_msg, "bgr8")->image;
      for (auto &gate : gate_line_buffer_[29])
      {
        cv::Mat rvec, tvec;
        cv::rectangle(show, gate[0], gate[1], cv::Scalar(255, 0, 0), 2, 8, 0);
      }
      sensor_msgs::ImagePtr msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", show).toImageMsg();
      pubImage_.publish(msg);
    }

    findGate(image_msg);
  }
}

void Sub8StartGateDetector::findGate(const sensor_msgs::ImageConstPtr &image_msg)
{
  cv::Mat cvImageMat = cv_bridge::toCvShare(image_msg, "bgr8")->image;

  cols_ = cvImageMat.cols;
  rows_ = cvImageMat.rows;

  // Get mean and deviation of the color in the image
  cv::Scalar mean;
  cv::Scalar stddev;
  cv::meanStdDev(cvImageMat, mean, stddev);

  // Threshold the image using mean and 2 standard deviations
  cv::Mat output;
  cv::inRange(cvImageMat, mean - 2 * stddev, mean + 2 * stddev, output);
  cv::bitwise_not(output, output);
  cv::dilate(output, output, cv::Mat(), cv::Point(-1, -1), 2, 1, 1);

  // Get a skeleton of things in the thresholded image
  cv::Mat skel(output.size(), CV_8UC1, cv::Scalar(0));
  cv::Mat temp(output.size(), CV_8UC1);
  cv::Mat element = cv::getStructuringElement(cv::MORPH_CROSS, cv::Size(3, 3));
  bool done = false;
  do
  {
    cv::morphologyEx(output, temp, cv::MORPH_OPEN, element);
    cv::bitwise_not(temp, temp);
    cv::bitwise_and(output, temp, temp);
    cv::bitwise_or(skel, temp, skel);
    cv::erode(output, output, element);

    double max;
    cv::minMaxLoc(output, 0, &max);
    done = (max == 0);
  } while (!done);

  // Use HoughLine to find lines from the skeleton image
  cv::Mat cdst = cv::Mat::zeros(cvImageMat.size(), cvImageMat.type());
  std::vector<cv::Vec4i> lines;
  cv::HoughLinesP(skel, lines, 1, CV_PI / 180, 70, 50, 20);
  for (size_t i = 0; i < lines.size(); i++)
  {
    cv::Vec4i l = lines[i];
    cv::line(cdst, cv::Point(l[0], l[1]), cv::Point(l[2], l[3]), cv::Scalar(0, 0, 255), 3, CV_AA);
  }

  // Dilate the image a little and find contours
  cv::dilate(cdst, cdst, cv::Mat(), cv::Point(-1, -1), 2, 1, 1);
  cv::Mat canny_output = cv::Mat::zeros(cvImageMat.size(), cvImageMat.type());
  std::vector<std::vector<cv::Point>> contours;
  std::vector<cv::Vec4i> hierarchy;
  cv::cvtColor(cdst, canny_output, CV_BGR2GRAY);
  cv::Canny(canny_output, canny_output, 100, 100 * 2, 3);
  cv::findContours(canny_output, contours, hierarchy, CV_RETR_TREE, CV_CHAIN_APPROX_SIMPLE, cv::Point(0, 0));

  // Find occupying rectangles and save the ones that are within the tolerance
  // size
  std::vector<cv::Rect> minRect;
  for (auto &contour : contours)
  {
    cv::Rect rect = cv::boundingRect(cv::Mat(contour));
    if (rect.area() > START_GATE_SIZE_TOLERANCE)
    {
      minRect.push_back(rect);
    }
  }

  // Add the points to the buffer
  std::vector<std::vector<cv::Point2f>> rectanglePoints;
  for (auto &rect : minRect)
  {
    rectanglePoints.push_back(std::vector<cv::Point2f>({ rect.tl(), rect.br() }));
  }

  gate_line_buffer_.push_back(rectanglePoints);
}

double Sub8StartGateDetector::getGateDistance()
{
  if (gate_line_buffer_.size() < 15)
  {
    return -1;
  }
  double x = (START_GATE_WIDTH * cam_model_.fx()) / mean(accSizeX_);
  double y = (START_GATE_HEIGHT * cam_model_.fy()) / mean(accSizeY_);
  return (x + y) / 2;
}

bool Sub8StartGateDetector::requestStartGatePosition2d(sub8_msgs::VisionRequest2D::Request &req,
                                                       sub8_msgs::VisionRequest2D::Response &resp)
{
  // This was called too soon and doesn't have enough in the buffer
  if (gate_line_buffer_.size() < 15)
  {
    resp.found = false;
    return true;
  }

  resp.pose.x = mean(accX_);
  resp.pose.y = mean(accY_);
  resp.pose.theta = 0;

  resp.camera_info = cam_model_.cameraInfo();
  resp.found = true;
  return true;
}

bool Sub8StartGateDetector::requestStartGateEnable(std_srvs::SetBool::Request &req, std_srvs::SetBool::Response &resp)
{
  running_ = req.data;
  resp.success = true;
  resp.message = "Set start_gate running boolean";
  if (!req.data)
  {  // If false we should also clear the buffer
    gate_line_buffer_.clear();
  }
  return true;
}

bool Sub8StartGateDetector::requestStartGateDistance(sub8_msgs::BMatrix::Request &req,
                                                     sub8_msgs::BMatrix::Response &resp)
{
  resp.B = std::vector<double>{ getGateDistance() };
  return true;
}
