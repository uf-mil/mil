#include <navigator_vision_lib/color_tracker.hpp>
bool ColorTracker::track(cv::Mat frame_left, std::vector<cv::Point2f> points_small, double image_proc_scale, std::vector<char>& colors){

  double reset_scaling = 1 / image_proc_scale;
  std::vector<cv::Point> mypoints;


  cv::Mat draw = frame_left.clone();
  for (size_t i = 0; i < points_small.size(); i++)
  {
      cv::Point2d pt_L = points_small[ i ];
      pt_L = pt_L * reset_scaling;
      mypoints.push_back(pt_L);
      cv::circle(draw, pt_L, 5, cv::Scalar(0,0,0), -1);


  }
//  cv::imshow("hi1", draw);
//  cv::waitKey(33);

      cv::Point pts[1][4];
      pts[0][0] = mypoints[0];
      pts[0][1] = mypoints[1];
      pts[0][2] = mypoints[2];
      pts[0][3] = mypoints[3];

      const cv::Point* points[1] = {pts[0]};
      int npoints = 4;

      // Create the mask with the polygon
      cv::Mat1b mask(frame_left.rows, frame_left.cols, uchar(0));
      cv::fillPoly(mask, points, &npoints, 1, cv::Scalar(255));
//      cv::imshow("hi", mask);
//      cv::waitKey(33);

      // Compute the mean with the computed mask

      //frame_left.setTo(cv::Scalar(255,255,255));

      cv::Scalar average = cv::mean(frame_left, mask);

      // std::cout << average << std::endl;


}

void ColorTracker::clear(){

}
