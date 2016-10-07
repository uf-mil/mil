#include "ShapeFind.h"

ShapeFind::ShapeFind(std::string color) { parseColor = color; }
void ShapeFind::FindContours() {
  contoursfindMat.clear();
  hierarchyfindMat.clear();
  findContours(binary_frame, contoursfindMat, hierarchyfindMat, CV_RETR_TREE,
               CV_CHAIN_APPROX_SIMPLE, Point(0, 0));
}
void ShapeFind::FindShapes() {
  shapes.clear();
  for (int i = 0; i < contoursfindMat.size(); i++) {
    std::vector<cv::Point> approx;
    approxPolyDP(Mat(contoursfindMat[i]), approx,
                 arcLength(Mat(contoursfindMat[i]), true) * 0.025, true);
    shapes.push_back(approx);
  }
}
Point ShapeFind::findCenter(std::vector<Point> points) {
  int x = 0, y = 0;
  for (int i = 0; i < points.size(); i++) {
    x += points[i].x;
    y += points[i].y;
  }
  x /= points.size();
  y /= points.size();
  return Point(x, y);
}
void ShapeFind::GetSymbols(Mat frame, navigator_msgs::DockShapes *symbols) {
  binary_frame = frame.clone();
  if (binary_frame.empty()) {
    std::cerr << "frame too small" << std::endl;
    return;
  }
  FindContours();
  FindShapes();

#ifdef DO_SHAPE_DEBUG
  std::cout << "===== FRAME " << parseColor << "=====" << std::endl;
#endif
  for (int i = 0; i < shapes.size(); i++) {
    if (ShapeDetector::isCross(shapes[i])) {
      symbols->list.push_back(
          fillSymbolMsg(shapes[i], navigator_msgs::DockShape::CROSS));
    } else if (ShapeDetector::isTriangle(shapes[i])) {
      symbols->list.push_back(
          fillSymbolMsg(shapes[i], navigator_msgs::DockShape::TRIANGLE));
    } else if (ShapeDetector::isCircle(shapes[i])) {
      symbols->list.push_back(
          fillSymbolMsg(shapes[i], navigator_msgs::DockShape::CIRCLE));
    }
  }
}

navigator_msgs::DockShape ShapeFind::fillSymbolMsg(std::vector<cv::Point> shape,
                                                   std::string symbol) {
  navigator_msgs::DockShape hold;
  Point center = findCenter(shape);
  hold.CenterX = center.x;
  hold.CenterY = center.y;
  hold.Color = parseColor;
  hold.Shape = symbol;
  hold.img_width = binary_frame.cols;
  for (int j = 0; j < shape.size(); j++) {
    geometry_msgs::Point p;
    p.x = shape[j].x;
    p.y = shape[j].y;
    p.z = 0;
    hold.points.push_back(p);
  }
  return hold;
}
