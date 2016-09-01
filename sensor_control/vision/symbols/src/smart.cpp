#include "std_msgs/String.h"

#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
#include <ros/ros.h>
#include <sensor_msgs/image_encodings.h>
#include "FrameProc.h"
#include "ShapeFind.h"

#include <vector>

#include "DebugWindow.h"
#include "opencv2/opencv.hpp"

#include "navigator_msgs/GetDockShape.h"
#include "navigator_msgs/DockShapes.h"
#include "std_srvs/SetBool.h"

class ImageSearcher {
 private:
  static const int SHAPE_BUFFER_SIZE = 30;
  static const int POSSIBLE_SYMBOLS_SIZE = 9;
  static const int MAX_SEEN_GAP_SEC = 5;
  static const int MAX_SEEN_GAP_NSEC = 0;
  ros::Duration max_seen_gap_dur;
  ros::NodeHandle n;
  ros::Subscriber foundShapesSubscriber;
  ros::ServiceServer getShapeService;
  ros::ServiceServer runService;
  struct ShapesBuffer
  {
    std::vector<navigator_msgs::DockShape> buffer;
    std::string Shape;
    std::string Color;
    ros::Time lastSeen;
  };
  navigator_msgs::DockShapes syms;                          // latest frame
  ShapesBuffer foundShapes[POSSIBLE_SYMBOLS_SIZE];
  std::string possibleShapes[3] = {navigator_msgs::DockShape::CROSS, navigator_msgs::DockShape::TRIANGLE,
                                  navigator_msgs::DockShape::CIRCLE};
  std::string possibleColors[3] = {navigator_msgs::DockShape::RED, navigator_msgs::DockShape::BLUE, navigator_msgs::DockShape::GREEN};
  int frames;
  bool active;
 public:
  ImageSearcher() : 
    max_seen_gap_dur(MAX_SEEN_GAP_SEC,MAX_SEEN_GAP_NSEC)
  {
    int u = 0;
    for (int i = 0; i < 3; i++) {
      for (int j = 0; j < 3; j++) {
        ShapesBuffer sb;
        sb.Shape = possibleShapes[i];
        sb.Color = possibleColors[j];
        foundShapes[u] = sb;
        u++;
      }
    }
    foundShapesSubscriber = n.subscribe("/dock_shapes/found_shapes", 1000, &ImageSearcher::foundShapesCallback, this);
    runService = n.advertiseService("/dock_shapes/run", &ImageSearcher::runCallback, this);
    getShapeService = n.advertiseService("/dock_shapes/GetShape", &ImageSearcher::getShapeCallback, this);
  }
  bool validRequest(navigator_msgs::GetDockShape::Request &req)
  {
    return (req.Color == navigator_msgs::GetDockShape::Request::BLUE || req.Color == navigator_msgs::GetDockShape::Request::GREEN || req.Color == navigator_msgs::GetDockShape::Request::RED) 
      &&   (req.Shape == navigator_msgs::GetDockShape::Request::CIRCLE || req.Shape == navigator_msgs::GetDockShape::Request::CROSS || req.Shape == navigator_msgs::GetDockShape::Request::TRIANGLE);
  }
  float mean(int val, int size) { return val / size; }
  void shapeChecker(const navigator_msgs::DockShapes &symbols) {
    if (!active) return;
    for (int i = 0; i < symbols.list.size(); i++) {
      for (int k = 0; k < POSSIBLE_SYMBOLS_SIZE; k++) {
        if (symbols.list[i].Shape == foundShapes[k].Shape && symbols.list[i].Color == foundShapes[k].Color) {
          ros::Time now = ros::Time::now();
          if ( (now - foundShapes[k].lastSeen) > max_seen_gap_dur)
          {
            foundShapes[k].buffer.clear();
          }
          if (foundShapes[k].buffer.size() == SHAPE_BUFFER_SIZE)
            foundShapes[k].buffer.pop_back();
          foundShapes[k].buffer.insert(foundShapes[k].buffer.begin(),symbols.list[i]);
          foundShapes[k].lastSeen = now;
          std::cout << "Color=" << foundShapes[k].Color << " Shapes=" << foundShapes[k].Shape << " Buf=" << foundShapes[k].buffer.size() << std::endl;
        }
      }
    }
  }
  navigator_msgs::DockShape getAverageShape(ShapesBuffer* sb)
  {
    //Do cool stats stuff to get rid of outliers, calculte mean center
    return sb->buffer[0];
  }
  void foundShapesCallback(const navigator_msgs::DockShapes &symbols) {
    if (!active) return;
    frames++;
    syms = symbols;
    shapeChecker(symbols);
  }

  bool getShapeCallback(navigator_msgs::GetDockShape::Request &req, navigator_msgs::GetDockShape::Response &res) {
    if (!active) {
        res.found = false;
        res.error = navigator_msgs::GetDockShape::Response::NODE_DISABLED;
        return true;
    }
    if (!validRequest(req))
    {
      res.found = false;
      res.error = navigator_msgs::GetDockShape::Response::INVALID_REQUEST;
      return true;
    }
    if (frames < 10) {
      res.found=false;
      res.error = navigator_msgs::GetDockShape::Response::TOO_SMALL_SAMPLE;
      return true;
    }
    for (int i = 0; i < POSSIBLE_SYMBOLS_SIZE; i++) {
      if (req.Shape == foundShapes[i].Shape && req.Color == foundShapes[i].Color) {
        if (foundShapes[i].buffer.size() < SHAPE_BUFFER_SIZE)
        {
          res.found = false;
          res.error = navigator_msgs::GetDockShape::Response::SHAPE_NOT_FOUND;
          return true;         
        }
        res.found = true;
        res.symbol = getAverageShape(&foundShapes[i]);
        return true;
      }
    }
    res.found = false;
    res.error = navigator_msgs::GetDockShape::Response::SHAPE_NOT_FOUND;
    return true;
  }

  bool runCallback(std_srvs::SetBool::Request &req, std_srvs::SetBool::Response &res) {
    std_srvs::SetBool msg;
    msg.request.data = req.data;
    active = req.data;
    ros::service::call("/dock_shape_processor/run", msg);
    res.success = true;
    return true;
  }
};

int main(int argc, char **argv) {
  ros::init(argc, argv, "dock_shape_filter");
  ImageSearcher imageSearcher;
  while (waitKey(50) == -1 && ros::ok()) {
    ros::spinOnce();
  }
  ros::spin();
  return 0;
}
