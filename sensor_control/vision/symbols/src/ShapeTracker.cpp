#include "ShapeTracker.h"
ShapeTracker::ShapeTracker() : active(false)
{
  
}
void ShapeTracker::setActive(bool a)
{
  active = a;
}
void ShapeTracker::init(ros::NodeHandle& nh)
{
  getShapesService = nh.advertiseService("/dock_shapes/GetShapes", &ShapeTracker::getShapesCallback, this);
  allFoundShapesPublish = nh.advertise<navigator_msgs::DockShapes>("/dock_shapes/filtered_shapes", 10); 
}
void ShapeTracker::addShape(navigator_msgs::DockShape& s)
{
 for (auto tracked = shapes.begin(); tracked != shapes.end(); tracked++)
 {
  if (tracked->update(s)) return;
 }
 // ~printf("Inserting Shape \n");
 shapes.push_back(TrackedShape(s));
}
void ShapeTracker::addShapes(navigator_msgs::DockShapes& newShapes)
{
 // ~printf("Adding %d shapes current size = %d\n",newShapes.list.size(),shapes.size());
 for (auto shape = newShapes.list.begin(); shape != newShapes.list.end(); shape++)
 {
   addShape(*shape);
 }
 shapes.erase(std::remove_if(shapes.begin(), shapes.end(), [] (TrackedShape& s) {return s.isStale();}),shapes.end());
 // ~printf("Shapes size %d\n",shapes.size());
 navigator_msgs::DockShapes found_shapes;
 for (auto tracked = shapes.begin(); tracked != shapes.end(); tracked++)
  {
    if (tracked->isReady() )
    {
      found_shapes.list.push_back(tracked->get());
    }
  }
  allFoundShapesPublish.publish(found_shapes);
}
bool ShapeTracker::getShapesCallback(navigator_msgs::GetDockShapes::Request &req,
                       navigator_msgs::GetDockShapes::Response &res) {
  if (!active) {
    res.found = false;
    res.error = navigator_msgs::GetDockShapes::Response::NODE_DISABLED;
    return true;
  }
  if (!validRequest(req.Color,req.Shape)) {
    res.found = false;
    res.error = navigator_msgs::GetDockShapes::Response::INVALID_REQUEST;
    return true;
  }
  for (auto tracked = shapes.begin(); tracked != shapes.end(); tracked++)
  {
    if (tracked->sameType(req.Color,req.Shape) && tracked->isReady() )
    {
      res.shapes.list.push_back(tracked->get());
    }
  }
  if (res.shapes.list.size() > 0) res.found = true;
  else res.error =  navigator_msgs::GetDockShapes::Response::SHAPE_NOT_FOUND;
  return true;
}
bool ShapeTracker::validRequest(string& color, string& shape)
{
return (color == navigator_msgs::GetDockShape::Request::BLUE ||
        color == navigator_msgs::GetDockShape::Request::GREEN ||
        color == navigator_msgs::GetDockShape::Request::RED ||
        color == navigator_msgs::GetDockShape::Request::ANY) &&
       (shape == navigator_msgs::GetDockShape::Request::CIRCLE ||
        shape == navigator_msgs::GetDockShape::Request::CROSS ||
        shape == navigator_msgs::GetDockShape::Request::TRIANGLE ||
        shape == navigator_msgs::GetDockShape::Request::ANY);
}
