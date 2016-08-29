#include "DebugWindow.h"

#ifdef DO_DEBUG
Mat DebugWindow::color_frame = Mat();
navigator_msgs::DockShapes DebugWindow::symbols = navigator_msgs::DockShapes();

void DebugWindow::init() {
    color_frame = Mat();
    symbols = navigator_msgs::DockShapes();
    namedWindow("Result",CV_WINDOW_AUTOSIZE);
    std::cout << "Running GUI Debug mode" << std::endl;
}

void DebugWindow::UpdateColor(Mat &frame) {
    color_frame = frame;
}
void DebugWindow::UpdateResults(navigator_msgs::DockShapes symbols) {	
  Mat res = color_frame;
  for (int i = 0; i < symbols.list.size(); i++) {
    cv::Rect p = cv::Rect(symbols.list[i].CenterX-50, symbols.list[i].CenterY-50, 100, 50);
    cv::rectangle(res, p, cv::Scalar(200, 200, 200), -1);
    Scalar color;
    if(symbols.list[i].Color == navigator_msgs::DockShape::RED) color = Scalar(0,0,100);
    else if(symbols.list[i].Color == navigator_msgs::DockShape::GREEN) color = Scalar(0,100,0);
    else if(symbols.list[i].Color == navigator_msgs::DockShape::BLUE) color = Scalar(100,0,0);
    
    
    if(symbols.list[i].Shape == navigator_msgs::DockShape::TRIANGLE) putText(res, "TRI", Point(symbols.list[i].CenterX-25, symbols.list[i].CenterY-25), 4, 1, color,  3);
    if(symbols.list[i].Shape == navigator_msgs::DockShape::CIRCLE) putText(res, "CIR", Point(symbols.list[i].CenterX-25, symbols.list[i].CenterY-25), 4, 1, color,  3);
    if(symbols.list[i].Shape == navigator_msgs::DockShape::CROSS) putText(res, "CROSS", Point(symbols.list[i].CenterX-50, symbols.list[i].CenterY-25), 4, 1, color,3);
  }
  imshow("Result",res);
}
#endif
