# Dock Shapes
This package contains executables for finding and identifying the symbols used in the Detect and Deliver & Identify Symbols and Dock challenges

## shape_finder
shape_finder uses OpenCV to look for pottential shapes in a ros image. When a pottential shape is found, it is published to /dock_shapes/found_shapes
The area of the frame shape_finder looks in can be set through /dock_shapes_finder/setROI

### Technical
The shapes are found with the following method:

1. Frame is cropped to the region of interest
2. Frame is converted to grayscale
3. Edges are discovered with Canny Edge Detector
4. Contours are found in the edges
5. Contours that are very small or oddly colored (very white, clouds) are removed
6. Of the remaining contours, geometric tests are applied to find circles,triangles,and cruciforms
7. For each valid shape found, the average color of the contour is taken to determine if it is closest to red, green, or blue

## shape_filter
shape_filter subscribes to ```/dock_shapes/found_shapes``` and tries to filter out incorrectly identified shapes or outliers.

shape_filter provides the ```/dock_shapes/GetShape``` and ```/dock_shapes/GetShapes``` services for other nodes to receive data of likely shapes found

### Technical
* For each type of symbol (Ex: GREEN CIRCLE), shape_filter keeps a buffer of 10 recently found shapes
* When a new shape is found, it is added to the buffer
* If it has been greater than a certain duration since that shape was last found, the buffer is cleared before adding the new one
* When the buffer becomes full, outliers in the buffer are removed using statistics tests (if center is x std.devs from mean center)
* When a service request comes in, the most recent shape of that type is returned in the buffer is full (many of that type have been found recently)
statistics)
* If the buffer is not full (not many found in short time period or many outliers), that shape is not returned

## ray_lidar
ray_lidar provides services for finding where a camera pixel is in 3D space using Navigator's velodyne lidar.

### Technical
* ray_lidar subscribes to the lidar and caches recent messages and the camerainfo topic specified in the launch file
* when a service arrives to transform a pixel to 3D, it selects a lidar message near the time specified in the header of the service request
* It loops through every point in the lidar pointcloud, projecting it into the camera frame using the calibration from the camera info matrix
* For projected points that are within the camera frame, the distance between the requested point and each point is calculated (in 2D)
* Points whose distance is less than the tollerance specified in the request and appended to the response
* A normal to these points is estimated and added to the response


# Usage
1. Built the project ```catkin_make navigator_shoot_vision shape_finder shape_filter```
1. Launch the two executables ```roslaunch navigator_shoot_vision dock_shapes.launch```
2. Call the run service ```rosservice call /dock_shapes/run "{data: true}"```
4. View the data produced through the debug images, services, or topics listed below

## Debug
The shape_finder executable provides ros image topics for debugging, which can be disabled with the compiler flag DO_DEBUG
/dock_shapes/finder/debug_color Shows the camera frame with the identified shapes drawn onto it
/dock_shapes/finder/debug_contours Shows the contours detected in the frame
/dock_shapes/raylider/markers Shows the transformed lidar pointcloud to camera frame with a normal pointing away from the plane
/camera_lidar_transformer/points_debug Image topic showing the projected lidar points into the camera frame

## Services Provided

Service | Type | Behavior 
--- | --- | --- 
/dock_shapes/GetShape | navigator_msgs/GetDockShape | Returns the center and points of the specified shape if found
/dock_shapes/GetShapes | navigator_msgs/GetDockShapes | Returns all found shapes in the frame
/dock_shapes/run | std_srvs/SetBool | Starts/stop the vision services
/dock_shapes_finder/setROI | navigator_msgs/SetROI | Sets the region of interest in the camera frame to look for shapes
/camera_lidar_transformer/transform_camera | navigator_msgs/CameraToLidarTransform | Uses lidar to find nearby 3D points and a normal from a pixel in a camera

## Topics provided
Topic | Type | Behavior 
--- | --- | --- 
/dock_shapes/found_shapes | navigator_msgs/DockShapes | Returns the center and points of the specified shape if found

## Config
Many of the constants used in the package can be changed at runtime using the ros paramaters found in shape_finder.yaml, shape_filter.yaml and dock_shapes.launch
Below are some of the most important parameters:

Param | Type | Usage
--- | --- | --- 
symbol_camera | string | The image topic for shape_finder to subscribe to and look for shapes in
image_tranport | string | To set type of image transport (raw,compressed,color)
auto_start | bool | Sets the vision nodes to start looking for shapes at launch or wait for call to run service
