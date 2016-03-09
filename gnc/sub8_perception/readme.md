Sub8 PCL
========

PCL/CV object identification, recognition and registration.


# Buoy Detector

The buoy detector does simple OpenCV thresholding to find the buoy in a 2D HSV image. The centroid of the detected contour is projected into the point cloud for 3D analysis (which is where the heavy lifting takes place). More work is being done (by Tess) to make the vision component more robust.

A service is called, and this returns a pose that is the estimated pose of the buoy in the camera stereo frame. When the service is not called, this node simply caches incoming point cloud and image data until it is needed to compute a buoy position.

To handle potential timing discrepancy, the timestamp at which the images and pointclouds were recorded is carried through the messages, so that the TF cache can be used in the frame-lookup, and we don't have any loss in data. Instead of looking up `buoy->cameras->base_link` and hoping the movement during propagation time was small, we can do `buoy->cameras->base_link[10 seconds ago]` i.e. look up the transform for when the image was taken.

##### Notes

* Sphere fitting won't work, because buoys are fairly specular, and cause silly artifacts in stereo reconstruction, making the buoys very non-spherical in the pt cloud.

* The buoy detector will give back an estimated center, that is not necessarily the actual center of the buoy.

##### TODO

* TF integration (On mission side)
* Better documentation for pcl_tools
* Add Tess' buoy segmentation method
* Robustify for using on other vision tasks
* Bayes filter buoy position for outlier removal
* Implement more than one color
* Add more visualization tools
* Determine the physical center of the buoy

##### Usage

```shell
    # running
    rosrun sub8_perception pcl_buoy

    # requesting results
    rosservice call /vision/buoys/red "target_name: 'red'"
```

A nice trick I use to call the service really frequently is

```shell
watch -n 0.1 "rosservice call /vision/buoys/red \"target_name: 'red'\""
```

##### Visualizing

In Rviz, make sure you have tf to whatever frame the cameras are broadcasting in. `roslaunch sub8_launch tf.launch` does that to base_link for the current setup.
Add a "Marker" visualizer, and set its topic to `/visualization/buoys`. Buoy detection (and therefore visualization) will only be performed if the service is called.

If you are using a bag

```shell
    ROS_NAMESPACE=/stereo/left rosrun image_proc image_proc
    rosbag play ./holding_buoy_mil.bag
```

