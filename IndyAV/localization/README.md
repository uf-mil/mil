# Localization
Software that determines the ego vehicles state given interial and extroperceptive sensors.

## Packages
Information about current included software will go here.

## Discussion
Currently we are investigating 3 types of sensor to be used used for localization:

### Inertial Navigator Sensors and GNSS (GPS, GLONASS, etc.)
INS and GNSS are usually coupled into an single device. 

#### Pros
* Directly yields position and orientation data, minimal computation required.
* RTK and other corrections can greatly boost the accuracy of the position measurement.

#### Cons
* GNSS data rate is slow < 10Hz
* Position can drift if too reliant on inertial sensors.
* GNSS is known to produce large errors if environmental conditions negatively affect the signals.

### LIDAR Localization

#### Pros
* High definition geometric data that can be compared to a know point cloud.
* Plenty of existing algorithms to do this: e.g NDT Matching
* Operates in almost any lighting conditions

#### Cons
* Slow data acquisition rate, at high speeds a mechanical spinning LIDAR would not capture the world correctly
* Somewhat costly, not as bad anyone
* Mechanical spinning LIDARS will be affected my the fast movements of the vehicle.

### Camera Localization
Camera based localization would rely on algorithm suchs as [ORB-SLAM](https://webdiis.unizar.es/~raulmur/orbslam/) to determine the position and orientation of the sensor with repsect to a pre-constructed map.
#### Pros
* Cheaper than LIDARs most of the time
* High data acquisition rate, 100+ FPS

#### Cons
* Camera based computations can be costly which would reduce the effectiveness of their high data acquisition
* Lighting can have an extremely affect the usuability of the sensor.