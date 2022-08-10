# LIDAR

One necessary instrument on NaviGator is its LIDAR system. This system is able
to generate a 3D PointCloud of the surrounding area.

## Simulation

Because the instrument is necessary for the boat to move, the component also
must be simulated in the Gazebo environment. This is done with the help of a few
tools.

The Virtual RobotX competition, pioneered by the Open Source Robotics Foundation,
launched with a set of tools for simulating boats on water. One of these tools
uses [another Gazebo plugin](https://bitbucket.org/DataspeedInc/velodyne_simulator/)
in order to create a simulated Velodyne LIDAR component. The VRX competition
tools provide a helpful model and component architecture, while the plugin
provides the functionality to generate the actual LIDAR beams.
