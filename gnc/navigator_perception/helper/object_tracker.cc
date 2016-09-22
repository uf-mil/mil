#include <navigator_vision_lib/object_tracker.hpp>


void ObjectTracker::begin_tracking_object(std::vector<cv::Point> points, cv::Mat left_frame){
      orig_frame = left_frame;
      for(cv::Point p: points)
        prev_points.push_back(p);
}

bool ObjectTracker::track_object(cv::Mat left_frame, std::vector<cv::Point2f>& points_2d){
      cv::Mat image_prev = orig_frame.clone();
      std::vector<cv::Point2f> points_next;
      std::vector<uchar> status;
      std::vector<float> err;

      // Find position of feature in new image
      cv::calcOpticalFlowPyrLK(
        image_prev, left_frame, // 2 consecutive images
        prev_points, // input point positions in first im
        points_next, // output point positions in the 2nd
        status,    // tracking success
       err      // tracking error
      );
      for(uchar e : status){
        if(e == 0){
          return false;
        }
      }

      for(int i = 0; i != points_next.size(); ++i){
        cv::Point a = points_next[i];
        cv::Point b;
        cv::Point c;
        bool b_init = true;
        // Get the furtherst point away from the current point
        int furthest_point = get_furthest_point(points_next, i);
        // Select the other to points, other than the current and the furthest away
        for(int j = 0; j != points_next.size(); ++j){
          if(j != furthest_point && j != i && b_init){
             b_init = false;
             b = points_next[j];
          }else if(j != furthest_point && j != i){
             c = points_next[j];
             break;
          }
        }
        // Get the angles between the vectors
        cv::Point e = b - a;
        cv::Point f = c - a;
        e *= 1/cv::norm(e);
        f *= 1/cv::norm(f);
        double h = e.dot(f);
        double t1 = acos(h);
        double diff = t1 - M_PI/2;
        if(fabs(diff) > M_PI/5) return false;
      }

      cv::Mat img_display;
      left_frame.copyTo( img_display );

      for(cv::Point p: points_next){
        cv::circle(img_display, p, 5, cv::Scalar(0,0,0), -1);

      }
      debug_image_tracking_points.publish(nav::convert_to_ros_msg("mono8", img_display));
      ros::spinOnce();

      prev_points = points_next;
      orig_frame = left_frame;
      points_2d = points_next;

  return true;

}

void ObjectTracker::clear(){
  prev_points.clear();

}

int get_furthest_point(std::vector<cv::Point2f> mymodel, int point){
  float max_val = 0;
  int max = -1;
  for(int j = 0; j != 4; ++j){
    if(j == point) continue;
    cv::Point diff = mymodel[point] - mymodel[j];
    float dist_from_starting = cv::norm(diff);
    if(dist_from_starting > max_val){
      max_val = dist_from_starting;
      max = j;
    }
  }
  return max;
}

