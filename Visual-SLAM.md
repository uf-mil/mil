Visual SLAM
===========

Right now, there are three parallel paths to visual navigation:

Ralph & David: Forward camera SLAM and stereo point cloud generation
David: Visual Odometry from conventional optical flow
Jake & Tess: MonoSLAM for down-camera

# Considerations
* Work in an underwater "outdoor" environment
* Handle occlusion due to opaqueness of water
* Handle occlusion in general
* Provide a motion estimate based on world knowledge, and not just integrating velocity (i.e. DVL)
* Generate a traversibility map
* Work in real-time

# Questions
* Do we need to enhance the simulator to support visual SLAM activities?
* Do we need better cameras?

# Notes
* Down-camera and forward camera SLAM are almost certainly different problems. Each works with remarkably different scene disparity, and the forward cameras work with much more occlusion than the down-camera