Point Cloud Labeling & Analysis Tools 
---
Point cloud labeling tool:
The point cloud labeling tool takes in a bag and outputs a bag. 

The input bag needs to have the following topics:
```
/velodyne_points
/stereo/right/image_raw
/stereo/right/camera_info
```

The output bag will have the following structure:
For each point cloud message in `/velodyne_points` in the **input bag** there is `PerceptionObjectArray` message in the **output bag**. Each `PerceptionObjectArray` message contains a list of objects for that point cloud. Also contained in the `PerceptionObjectArray` is a special object called the "spurious" object. This object contains all of the points for that point cloud that were not classified to any object. 

In order to run the labeling tool you must run:

```
rosrun point_cloud_object_detection_and_recognition pcodar_labeling_tool <input_bag> <output_bag>
```
