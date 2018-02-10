# GPS/INS Device
NaviGator uses an integrated multi-sensor board to estimate our position, orientation, and velocity in absolute terms on the earth. This board comes from [Sylphase](https://sylphase.com/technology.html), a company founded by a former student of the lab. Here's a description from their website:

> When connected to a computer running our SDGPS software, the system fuses GPS L1 observations with measurements from an accelerometer, gyroscope, magnetometer, and barometer into an optimal estimate of absolute position, relative position (odometry), orientation, velocity, and angular velocity.

The sylphase board and accompanying proprietary software can be viewed as a black box which produces our state values. **No sensor fusion, filtering, or other state estimation code is maintained by the NaviGator team** as this board simply solves this problem and has been demonstrated to be sufficiently precise and reliable.

# Global frames
The sylphase board reports position, orientation, and velocity (both linear and angular) in two frames. A third frame (LLA) can also be calculated from these states.

* ENU (East North Up): origin (0, 0, 0) is set to position of first GPS fix since starting the sylphase software. As the name suggests, the axes are aligned such that X, Y, and Z point East, North, and Up respectively. Published to ```/odom``` with tf frame ```enu```
* [ECEF (Earth Centered, Earth Fixed)](https://en.wikipedia.org/wiki/ECEF): origin (0, 0, 0) is fixed to the center of the earth, with the axes pointing along the earth-fixed IRP and IRM lines. Published to ```/absodom``` with tf frame ```ecef```
* [LLA (Longitude, Latitude, Altitude)](https://en.wikipedia/wiki/Geographic_coordinate_system#Geographic_latitude_and_longitude): is another absolute frame fixed to the earth used more often in navigation, maps, etc. The z component (altitude) points up from sea level so should be around 0 anytime NaviGator is in the water. It is not directly measured by the sylphase board, but can be calculated from a ECEF position.

# Global frame conversions
Is is possible to calculate conversion from any one of the global frames listed above to the other 2. Because ENU is relative to NaviGator's position when the GPS software is started, a pair of synchronized ENU and ECEF messages is needed to go from ENU to ECEF/LLA or visa/versa. Conversion between the two absolute frames can be done without an initial message pair. For example, given an longitude, latitude

Implementations of the math to do these conversions can be found in the [rawgps_common python library in mil_common](https://github.com/uf-mil/mil_common/blob/master/gnc/rawgps_common/src/rawgps_common/gps.py).

There is also a node run on NaviGator, [coordinate_conversion_server.py](https://github.com/uf-mil/NaviGator/blob/master/utils/navigator_tools/nodes/coordinate_conversion_server.py), which subscribes to ```/odom``` and ```/absodom``` in order to publish the current longitude latitude and altitude to ```/lla```. The node also provides a service ```/convert``` which will perform a conversion of n points in any of the 3 global frames to another.

For example, the global bounds server uses the ```/convert``` service to allow bounds to be set in LLA coordinates but be correctly represented in the occupancy grid, which is in ```enu``` frame.

