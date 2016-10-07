#include "FrameProc.h"

int FrameProc::blur_size = 7;
FrameProc::FrameProc() {
  red = ColorThresh{Scalar(0, 10, 100), Scalar(30, 255, 255)};
  red2 = ColorThresh{Scalar(155, 10, 100), Scalar(180, 255, 255)};
  blue = ColorThresh{Scalar(90, 100, 100), Scalar(150, 255, 255)};
  green = ColorThresh{Scalar(30, 85, 25), Scalar(85, 255, 255)};

#ifdef DO_DEBUG
  namedWindow("blured", CV_WINDOW_AUTOSIZE);
  // namedWindow("hsv",CV_WINDOW_AUTOSIZE);
  namedWindow("blue", CV_WINDOW_AUTOSIZE);
  namedWindow("green", CV_WINDOW_AUTOSIZE);
  namedWindow("red", CV_WINDOW_AUTOSIZE);
  createTrackbar("Blur Size", "blured", &blur_size, 100);
#endif
}
void FrameProc::init(ros::NodeHandle& nh) {
  // Set HSV values
  nh.getParam("hsv/red1/low/H", red.low[0]);
  nh.getParam("hsv/red1/low/S", red.low[1]);
  nh.getParam("hsv/red1/low/V", red.low[2]);
  nh.getParam("hsv/red1/high/H", red.high[0]);
  nh.getParam("hsv/red1/high/S", red.high[1]);
  nh.getParam("hsv/red1/high/V", red.high[2]);

  nh.getParam("hsv/red2/low/H", red2.low[0]);
  nh.getParam("hsv/red2/low/S", red2.low[1]);
  nh.getParam("hsv/red2/low/V", red2.low[2]);
  nh.getParam("hsv/red2/high/H", red2.high[0]);
  nh.getParam("hsv/red2/high/S", red2.high[1]);
  nh.getParam("hsv/red2/high/V", red2.high[2]);

  nh.getParam("hsv/blue/low/H", blue.low[0]);
  nh.getParam("hsv/blue/low/S", blue.low[1]);
  nh.getParam("hsv/blue/low/V", blue.low[2]);
  nh.getParam("hsv/blue/high/H", blue.high[0]);
  nh.getParam("hsv/blue/high/S", blue.high[1]);
  nh.getParam("hsv/blue/high/V", blue.high[2]);

  nh.getParam("hsv/green/low/H", green.low[0]);
  nh.getParam("hsv/green/low/S", green.low[1]);
  nh.getParam("hsv/green/low/V", green.low[2]);
  nh.getParam("hsv/green/high/H", green.high[0]);
  nh.getParam("hsv/green/high/S", green.high[1]);
  nh.getParam("hsv/green/high/V", green.high[2]);

  // Set blue/dilate size
  nh.getParam("blur_size", blur_size);
}
void FrameProc::ErodeDilate() {
  Mat new_frame;
  bilateralFilter(rgb_frame, new_frame, blur_size, blur_size * 2,
                  blur_size / 2);
  rgb_frame = new_frame;
  // medianBlur ( rgb_frame, rgb_frame,blur_size );;
  // erode(rgb_frame,rgb_frame,erode_element);
  // dilate(rgb_frame,rgb_frame,dilate_element);
}
void FrameProc::ConvertHSV() { cvtColor(rgb_frame, hsv_frame, CV_BGR2HSV); }
void FrameProc::ThresholdColors() {
  Mat rtemp, rtemp2;
  inRange(hsv_frame, red.low, red.high, rtemp);
  inRange(hsv_frame, red2.low, red2.high, rtemp2);
  binary_red_frame = rtemp | rtemp2;
  inRange(hsv_frame, blue.low, blue.high, binary_blue_frame);
  inRange(hsv_frame, green.low, green.high, binary_green_frame);

#ifdef DO_DEBUG
  imshow("blue", binary_blue_frame);
  imshow("green", binary_green_frame);
  imshow("red", binary_red_frame);
#endif
}
void FrameProc::Prepare(Mat& frame) {
  rgb_frame = frame;

#ifdef DO_DEBUG
  DebugWindow::UpdateColor(rgb_frame);
#endif
  ErodeDilate();
  ConvertHSV();
  ThresholdColors();
#ifdef DO_DEBUG
  imshow("blured", rgb_frame);
// imshow("hsv",hsv_frame);
#endif
}

Mat FrameProc::GetRed() { return binary_red_frame; }
Mat FrameProc::GetBlue() { return binary_blue_frame; }
Mat FrameProc::GetGreen() { return binary_green_frame; }
