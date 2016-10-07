#include "std_msgs/String.h"

#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
#include <ros/ros.h>
#include <sensor_msgs/image_encodings.h>
#include <boost/accumulators/accumulators.hpp>
#include <boost/accumulators/statistics.hpp>
#include <boost/circular_buffer.hpp>
#include <vector>
#include "ContourMethod/FrameProc.h"
#include "ContourMethod/ShapeFind.h"

#include "DebugWindow.h"
#include "opencv2/opencv.hpp"

#include <navigator_msgs/DockShapes.h>
#include <navigator_msgs/GetDockShape.h>
#include <navigator_msgs/GetDockShapes.h>
#include "std_srvs/SetBool.h"

class ShapesBuffer {
  static double MAX_X_VAR;
  static double MAX_Y_VAR;
  static double STD_DEV_THRESHOLD;
  static ros::Duration max_seen_gap_dur;
  boost::circular_buffer<navigator_msgs::DockShape> buffer;
  std::string Shape;
  std::string Color;
  void removeOutlierShapes() {
    // Do cool stats stuff to get rid of outliers
    using namespace boost::accumulators;
    accumulator_set<int, features<tag::mean, tag::variance>> xAcc;
    accumulator_set<int, features<tag::mean, tag::variance>> yAcc;
    for (auto shape : buffer) {
      xAcc(shape.CenterX);
      yAcc(shape.CenterY);
    }
    auto xMean = boost::accumulators::mean(xAcc);
    auto xSD = sqrt(boost::accumulators::variance(xAcc));
    auto yMean = boost::accumulators::mean(yAcc);
    auto ySD = sqrt(boost::accumulators::variance(yAcc));
    // ~printf("%s %s Xmean=%f Xvar=%f Ymean=%f
    // Yvar=%f\n",Color.c_str(),Shape.c_str(),xMean,xSD,yMean,ySD);
    // ~if (xSD > MAX_X_VAR || ySD > MAX_Y_VAR)
    // ~{
    // ~printf("%s %s CLEARING B/C TOO HIGH VAR",Color.c_str(),Shape.c_str());
    // ~clear();
    // ~return;
    // ~}
    auto new_end = std::remove_if(
        buffer.begin(), buffer.end(), [=](navigator_msgs::DockShape &shape) {
          double xstds = std::abs(shape.CenterX - xMean) / xSD;
          double ystds = std::abs(shape.CenterY - yMean) / ySD;
          if (xstds > STD_DEV_THRESHOLD &&
              std::abs(shape.CenterX - xMean) > 25) {
            // ~printf("%s %s REMOVING X OVER
            // STD.DEV\n",Color.c_str(),Shape.c_str());
            return true;
          } else if (ystds > STD_DEV_THRESHOLD &&
                     std::abs(shape.CenterY - yMean) > 25) {
            // ~printf("%s %s REMOVING Y OVER STD.DEV %f
            // %d\n",Color.c_str(),Shape.c_str(),yMean,shape.CenterY);
            return true;
          }
          return false;
        });
    buffer.erase(new_end, buffer.end());
  }

 public:
  ShapesBuffer() {}
  ShapesBuffer(std::string shape, std::string color, const int size)
      : Shape(shape), Color(color), buffer(size) {}
  static void init(ros::NodeHandle &nh) {
    nh.param<double>("max_x_var", MAX_X_VAR, 50);
    nh.param<double>("max_y_var", MAX_Y_VAR, 50);
    nh.param<double>("std_dev_threshold", STD_DEV_THRESHOLD, 1.5);
    double seconds;
    nh.param<double>("max_seen_gap_seconds", seconds, 0.5);
    max_seen_gap_dur = ros::Duration(0, seconds * 1000000000);
  }
  bool isStale() {
    if (buffer.empty()) return true;
    ros::Time now = ros::Time::now();
    return (now - buffer.back().header.stamp) > max_seen_gap_dur;
  }
  void insert(navigator_msgs::DockShape ds) {
    if (!buffer.empty()) {
      if (ds.header.stamp - buffer.back().header.stamp > max_seen_gap_dur)
        clear();
    }
    buffer.push_back(ds);
    if (buffer.full()) removeOutlierShapes();
  }
  bool getAverageShape(navigator_msgs::DockShape &shape) {
    if (!isFound()) return false;
    shape = buffer.back();
    return true;
  }
  bool sameType(navigator_msgs::DockShape shape) {
    return shape.Color == Color && shape.Shape == Shape;
  }
  bool sameType(std::string &shape, std::string &color) {
    return shape == Shape && color == Color;
  }
  bool isFound() { return buffer.full() && !isStale(); }
  void clear() { buffer.clear(); }
  std::string getColor() { return Color; }
  std::string getShape() { return Shape; }
};
double ShapesBuffer::MAX_X_VAR = 50;
double ShapesBuffer::MAX_Y_VAR = 50;
double ShapesBuffer::STD_DEV_THRESHOLD = 1.5;
ros::Duration ShapesBuffer::max_seen_gap_dur = ros::Duration(0, 500000000);

class ImageSearcher {
 private:
  static int SHAPE_BUFFER_SIZE;
  static const int POSSIBLE_SYMBOLS_SIZE = 9;
  ros::Duration max_seen_gap_dur;
  ros::NodeHandle n;
  ros::Subscriber foundShapesSubscriber;
  ros::ServiceServer getShapeService;
  ros::ServiceServer getShapesService;
  ros::ServiceServer runService;

  navigator_msgs::DockShapes syms;  // latest frame
  std::array<ShapesBuffer, POSSIBLE_SYMBOLS_SIZE> foundShapes;
  std::array<bool, POSSIBLE_SYMBOLS_SIZE> previousFound;
  std::string possibleShapes[3] = {navigator_msgs::DockShape::CROSS,
                                   navigator_msgs::DockShape::TRIANGLE,
                                   navigator_msgs::DockShape::CIRCLE};
  std::string possibleColors[3] = {navigator_msgs::DockShape::RED,
                                   navigator_msgs::DockShape::BLUE,
                                   navigator_msgs::DockShape::GREEN};
  int frames;
  bool active;

 public:
  ImageSearcher() : n("dock_shape_filter") {
    n.param<bool>("auto_start", active, false);
    n.param<int>("buffer_size", SHAPE_BUFFER_SIZE, 10);
    ShapesBuffer::init(n);
    int u = 0;
    for (int i = 0; i < 3; i++) {
      for (int j = 0; j < 3; j++) {
        ShapesBuffer sb(possibleShapes[i], possibleColors[j],
                        SHAPE_BUFFER_SIZE);
        foundShapes[u] = sb;
        u++;
        previousFound[u] = false;
      }
    }
    foundShapesSubscriber =
        n.subscribe("/dock_shapes/found_shapes", 1000,
                    &ImageSearcher::foundShapesCallback, this);
    runService = n.advertiseService("/dock_shapes/run",
                                    &ImageSearcher::runCallback, this);
    getShapeService = n.advertiseService(
        "/dock_shapes/GetShape", &ImageSearcher::getShapeCallback, this);
    getShapesService = n.advertiseService(
        "/dock_shapes/GetShapes", &ImageSearcher::getShapesCallback, this);
  }
  void reset() {
    frames = 0;
    for (auto foundShape = foundShapes.begin(); foundShape != foundShapes.end();
         foundShape++) {
      foundShape->clear();
    }
  }
  bool validRequest(navigator_msgs::GetDockShape::Request &req) {
    return (req.Color == navigator_msgs::GetDockShape::Request::BLUE ||
            req.Color == navigator_msgs::GetDockShape::Request::GREEN ||
            req.Color == navigator_msgs::GetDockShape::Request::RED) &&
           (req.Shape == navigator_msgs::GetDockShape::Request::CIRCLE ||
            req.Shape == navigator_msgs::GetDockShape::Request::CROSS ||
            req.Shape == navigator_msgs::GetDockShape::Request::TRIANGLE);
  }
  void shapeChecker(const navigator_msgs::DockShapes &symbols) {
    if (!active) return;
    for (auto newShape = symbols.list.begin(); newShape != symbols.list.end();
         newShape++) {
      for (int i = 0; i < foundShapes.size(); i++) {
        if (foundShapes[i].sameType(*newShape)) {
          foundShapes[i].insert(*newShape);
          // Notice when found state changed for logging
          bool found = foundShapes[i].isFound();
          if (found && !previousFound[i])
            printf("%s %s FOUND\n", foundShapes[i].getColor().c_str(),
                   foundShapes[i].getShape().c_str());
          if (!found && previousFound[i])
            printf("%s %s LOST\n", foundShapes[i].getColor().c_str(),
                   foundShapes[i].getShape().c_str());
          previousFound[i] = found;
        }
      }
    }
  }
  void foundShapesCallback(const navigator_msgs::DockShapes &symbols) {
    if (!active) return;
    frames++;
    shapeChecker(symbols);
  }

  bool getShapesCallback(navigator_msgs::GetDockShapes::Request &req,
                         navigator_msgs::GetDockShapes::Response &res) {
    for (auto shape = foundShapes.begin(); shape != foundShapes.end();
         shape++) {
      navigator_msgs::DockShape dockShape;
      if (shape->getAverageShape(dockShape))
        res.shapes.list.push_back(dockShape);
    }
    return true;
  }
  bool getShapeCallback(navigator_msgs::GetDockShape::Request &req,
                        navigator_msgs::GetDockShape::Response &res) {
    if (!active) {
      res.found = false;
      res.error = navigator_msgs::GetDockShape::Response::NODE_DISABLED;
      return true;
    }
    if (!validRequest(req)) {
      res.found = false;
      res.error = navigator_msgs::GetDockShape::Response::INVALID_REQUEST;
      return true;
    }
    if (frames < 10) {
      res.found = false;
      res.error = navigator_msgs::GetDockShape::Response::TOO_SMALL_SAMPLE;
      return true;
    }
    for (auto shape = foundShapes.begin(); shape != foundShapes.end();
         shape++) {
      if (shape->sameType(req.Shape, req.Color)) {
        if (shape->getAverageShape(res.symbol)) {
          res.found = true;
        } else {
          res.found = false;
          res.error = navigator_msgs::GetDockShape::Response::SHAPE_NOT_FOUND;
        }
        return true;
      }
    }
    res.found = false;
    res.error = navigator_msgs::GetDockShape::Response::SHAPE_NOT_FOUND;
    return true;
  }

  bool runCallback(std_srvs::SetBool::Request &req,
                   std_srvs::SetBool::Response &res) {
    if (!req.data && active)
      reset();  // If turning off while currently on, reset all the buffers
    std_srvs::SetBool msg;
    msg.request.data = req.data;
    active = req.data;
    ros::service::call("/dock_shape_finder/run", msg);
    res.success = true;
    return true;
  }
};
int ImageSearcher::SHAPE_BUFFER_SIZE = 10;

int main(int argc, char **argv) {
  ros::init(argc, argv, "dock_shape_filter");
  ImageSearcher imageSearcher;
  while (waitKey(50) == -1 && ros::ok()) {
    ros::spinOnce();
  }
  ros::spin();
  return 0;
}
