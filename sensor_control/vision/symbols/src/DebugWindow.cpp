#include "DebugWindow.h"

#ifdef DO_DEBUG
Mat DebugWindow::color_frame = Mat();
navigator_msgs::DockShapes DebugWindow::symbols = navigator_msgs::DockShapes();
void DebugWindow::init() {
  color_frame = Mat();
  symbols = navigator_msgs::DockShapes();
  namedWindow("Result", CV_WINDOW_AUTOSIZE);
  std::cout << "Running GUI Debug mode" << std::endl;
}

void DebugWindow::UpdateColor(Mat& frame) { color_frame = frame; }
void DebugWindow::UpdateResults(navigator_msgs::DockShapes& symbols) {
  Mat res = color_frame;
  for (navigator_msgs::DockShape symbol : symbols.list) {
    // cv::Rect p = cv::Rect(symbol.CenterX-50, symbol.CenterY-50, 100, 50);
    // cv::rectangle(res, p, cv::Scalar(200, 200, 200), -1);
    cv::circle(res, Point(symbol.CenterX, symbol.CenterY), 4,
               Scalar(255, 255, 255), 5);
    putText(res, symbol.Shape + "(" + symbol.Color + ")",
            Point(symbol.CenterX - 25, symbol.CenterY - 25), 4, 1,
            Scalar(0, 0, 0), 3);
  }

  imshow("Result", res);
}
#endif
