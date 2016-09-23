#include <navigator_vision_lib/color_tracker.hpp>

int get_color(cv::Scalar color) {
  auto blackness1 = color[0] - color[1];
  auto blackness2 = color[1] - color[2];
  if (fabs(blackness1) < 10 && fabs(blackness2) < 10) {
    return 0;
  }

  auto a = color[0];
  auto b = color[1];
  auto c = color[2];
  if (fabs(a - b) < 10 && b > c && a > c)
    return 1;
  else if (a > b && a > c)
    return 2;
  else if (b > a && b > c)
    return 3;
  else if (c > a && c > b)
    return 4;
  else
    return 0;
}

bool ColorTracker::track(cv::Mat frame_left,
                         std::vector<cv::Point2f> points_small,
                         double image_proc_scale) {
  debug_image_color.publish(nav::convert_to_ros_msg("bgr8", frame_left));
  ros::spinOnce();

  if (colors_found == 3) {
    mission_complete = 1;
    ROS_INFO("Mission Complete");

    for (std::string s : colors)
      std::cout << s << " ";
    return true;
  }

  double reset_scaling = 1 / image_proc_scale;
  std::vector<cv::Point> mypoints;

  cv::Mat draw = frame_left.clone();
  for (size_t i = 0; i < points_small.size(); i++) {
    cv::Point2d pt_L = points_small[i];
    pt_L = pt_L * reset_scaling;
    mypoints.push_back(pt_L);
    cv::circle(draw, pt_L, 5, cv::Scalar(0, 0, 0), -1);
  }

  cv::Point pts[1][4];
  pts[0][0] = mypoints[0];
  pts[0][1] = mypoints[1];
  pts[0][2] = mypoints[2];
  pts[0][3] = mypoints[3];

  const cv::Point *points[1] = {pts[0]};
  int npoints = 4;

  // Create the mask with the polygon
  cv::Mat1b mask(frame_left.rows, frame_left.cols, uchar(0));
  cv::fillPoly(mask, points, &npoints, 1, cv::Scalar(255));

  cv::Scalar average = cv::mean(frame_left, mask);
  int col = get_color(average);
  if (started) {
    bool changed = false;
    if (col != prev_color) {
      changed = true;
    }
    if (col == 0) {
      ROS_INFO("Turned Black");
      turned_black = true;
      colors_found = 0;
    } else if (turned_black && changed) {
      ++colors_found;
      std::stringstream s;
      s << "Found a  new color " << col << " " << average;
      ROS_INFO(s.str().c_str());

      if (col == 1)
        colors[colors_found - 1] = "y";
      else if (col == 2)
        colors[colors_found - 1] = "b";
      else if (col == 3)
        colors[colors_found - 1] = "g";
      else if (col == 4)
        colors[colors_found - 1] = "r";

    } else if (!changed) {
      std::stringstream s;
      s << "Same Color " << col << " " << average;
      ROS_INFO(s.str().c_str());

    } else if (!turned_black) {
      ROS_INFO("Waiting for black");
    }
  }
  last_color = average;
  prev_color = col;
  started = true;
  return false;
}

void ColorTracker::clear() {
  ROS_INFO("CLEAR");
  status = 0;
  mission_complete = 0;
  turned_black = 0;
  colors_found = 0;
  started = 0;
  prev_color = 0;
}

bool ColorTracker::check_status(
    navigator_msgs::ScanTheCodeMission::Request &req,
    navigator_msgs::ScanTheCodeMission::Response &resp) {
  resp.tracking_model = status;
  resp.mission_complete = mission_complete;

  if (!mission_complete) {
    return true;
  }
  std::vector<std::string> mycolors;
  mycolors.push_back(colors[0]);
  mycolors.push_back(colors[1]);
  mycolors.push_back(colors[2]);

  if (status) {
    resp.colors = mycolors;
  }

  return true;
}

void ColorTracker::set_status(bool i) { status = i; }
