#include "std_msgs/String.h"

#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
#include <ros/ros.h>
#include <sensor_msgs/image_encodings.h>
#include "FrameProc.h"
#include "ShapeFind.h"
#include <boost/circular_buffer.hpp>

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
  static const int MAX_SEEN_GAP_SEC = 0;
  static const int MAX_SEEN_GAP_NSEC = 500000000;
  ros::Duration max_seen_gap_dur;
  ros::NodeHandle n;
  ros::Subscriber foundShapesSubscriber;
  ros::ServiceServer getShapeService;
  ros::ServiceServer runService;
  struct ShapesBuffer
  {
    boost::circular_buffer<navigator_msgs::DockShape> buffer;
    std::string Shape;
    std::string Color;
    ros::Time lastSeen;
  };

  navigator_msgs::DockShapes syms;                          // latest frame
  std::array<ShapesBuffer, POSSIBLE_SYMBOLS_SIZE> foundShapes;
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
        sb.buffer = boost::circular_buffer<navigator_msgs::DockShape>(SHAPE_BUFFER_SIZE);
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
    for (auto newShape = symbols.list.begin(); newShape != symbols.list.end(); newShape++) {
      for (auto foundShape = foundShapes.begin(); foundShape  != foundShapes.end(); foundShape++) {
        if (newShape->Shape == foundShape->Shape && newShape->Color == foundShape->Color) {
          ros::Time now = ros::Time::now();
          if ( (now - foundShape->lastSeen) > max_seen_gap_dur)
          {
            foundShape->buffer.clear();
          }
          foundShape->buffer.push_back(*newShape);
          foundShape->lastSeen = now;
          std::cout << "Color=" << foundShape->Color << " Shapes=" << foundShape->Shape << " Buf=" << foundShape->buffer.size() << std::endl;
        }
      }
    }
  }
  
  
  navigator_msgs::DockShape getAverageShape(ShapesBuffer &sb)
  {
    //Do cool stats stuff to get rid of outliers, calculte mean center
    std::vector<int> x_centers;
    std::vector<int> y_centers;
    
    for (int i =0; i < SHAPE_BUFFER_SIZE; i++) {
        x_centers.push_back(sb.buffer[i].CenterX);
        y_centers.push_back(sb.buffer[i].CenterY);
    }
    std::sort(x_centers.begin(), x_centers.end());
    std::sort(y_centers.begin(), y_centers.end());
    
    int q1 = .25 * SHAPE_BUFFER_SIZE;
    int q3 = .75 * SHAPE_BUFFER_SIZE;
    
    int offX = 1.5 * (x_centers[q3] - x_centers[q1]);
    int offY = 1.5 * (y_centers[q3] - y_centers[q1]);
    
    int offXMin = x_centers[q1]-offX;
    int offXMax = x_centers[q3]+offX;
    int offYMin = y_centers[q1]-offY;
    int offYMax = y_centers[q3]+offX;
    
    std::vector<navigator_msgs::DockShape> fittedShapes;
    
    for (int i =0; i < SHAPE_BUFFER_SIZE; i++) {
        if(!(sb.buffer[i].CenterX > offXMax && sb.buffer[i].CenterX < offXMin && sb.buffer[i].CenterY > offYMax && sb.buffer[i].CenterY < offYMin))
            fittedShapes.push_back(sb.buffer[i]);
    }
    int meanX =0, meanY =0;
    for (int i =0; i < fittedShapes.size(); i++) {
        meanX+=fittedShapes[i].CenterX;
        meanY+=fittedShapes[i].CenterY;
    }
    meanX/=fittedShapes.size();
    meanY/=fittedShapes.size();
    
    //navigator_msgs::DockShape shape = *fittedShapes.end();
    //shape.CenterX = meanX;
    //shape.CenterY = meanY;
    std::cout<<meanX<<" "<<meanY<<std::endl;
    //return shape;
    return *sb.buffer.end();
  }
  void foundShapesCallback(const navigator_msgs::DockShapes &symbols) {
    if (!active) return;
    frames++;
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
    for (auto shape = foundShapes.begin(); shape  != foundShapes.end(); shape++) {
      if (req.Shape == shape->Shape && req.Color == shape->Color) {
        if (!shape->buffer.full())
        {
          res.found = false;
          res.error = navigator_msgs::GetDockShape::Response::SHAPE_NOT_FOUND;
          return true;         
        }
        res.found = true;
        res.symbol = getAverageShape(*shape);
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
