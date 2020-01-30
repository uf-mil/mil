#include <iostream>
#include <string>
#include <navigator_vision_lib/active_contours.hpp>
#include <navigator_tools.hpp>

using namespace std;
using namespace cv;

bool img_stuff(Mat& img_color, int* lo, int* hi, int* kernel_size)
{
  cout << "Image Processing" << endl;
  vector<Mat> hsv_split;
  Mat hue;
  cvtColor(img_color, hue, CV_BGR2HSV);
  split(hue, hsv_split);
  Mat blob_mask;
  Mat mask;
  inRange(hsv_split[0], Scalar(*lo), Scalar(*hi), blob_mask);
  int morph_elem = MORPH_ELLIPSE;
  int morph_size = *kernel_size;
  Mat element = getStructuringElement( morph_elem, Size( 2*morph_size + 1, 2*morph_size+1 ), Point( morph_size, morph_size ) );
  erode(blob_mask, mask, element);
  dilate(mask, mask, element);
  imshow("mask", mask);
  vector<vector<cv::Point2i>> contours;
  findContours(mask, contours, CV_RETR_LIST, CV_CHAIN_APPROX_NONE);
  Mat curve_img = Mat::zeros(mask.size(), CV_8U);
  Scalar color{255};
  nav::tools::sortContours(contours, false);
  if(contours.size() != 0)
  {
    drawContours(curve_img, contours, 0, color);
    cout << "Curve size: " << contours[0].size() << endl;
    for(auto& curve : contours)
    {
      cout << "Curve of size " << curve.size()  << " is " <<  (nav::ClosedCurve::validateCurve(curve)? "valid" : "invalid") << endl;
    }
  }
  imshow("curve", curve_img);
  return true;
}


int main(int argc, char** argv)
{
  cout << "This is Active Contours Test" << endl;
  nav::Perturbations::initPerturbationCache();

  int low_slider {0};
  int hi_slider {8};
  int low_max {255};
  int hi_max {255};
  int kernel_size {5};
  int k_sz_max {255};
  string win_name("Thresh");
  namedWindow(win_name);
  createTrackbar("hue_low", win_name, &low_slider, low_max);
  createTrackbar("hue_hi", win_name, &hi_slider, hi_max);
  createTrackbar("kernel_size", win_name, &kernel_size, k_sz_max);
  VideoCapture cap;
  if(!cap.open(0))
    return 0;
  for(;;)
  {
    Mat frame;
    cap >> frame;
    if(frame.empty())
      break;
    resize(frame, frame, Size(0,0), 0.5, 0.5);
    imshow("Thresh", frame);
    uint8_t wk = waitKey(1);
    cout << int(wk) << endl;
    img_stuff(frame, &low_slider, &hi_slider, &kernel_size);
    if(wk == 27)
      break;
  }
  cout << "I finished!" << nav::Perturbations::perturbation_cache.size() << endl;
  return 0;
}
