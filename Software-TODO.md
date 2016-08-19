##General Software TODO list

If you want to get involved this is a great place to start! If you're serious about a mission, put your name there so we know who is working on what. _Please note_ this list is far from comprehensive, so if you think of something that needs to be done add it!

### Command
* Mission runner __[MATT]__
* Network loss kill __[MATT]__

### Perception
* Calibrate Cameras
* Hydrophones __[DAVID]__
* Generate masks of mission component shapes __[DANIEL++]__
* Service to get pose from 2d mask __[DAVID]__
* Pointcloud from camera (do we even need to?) __[DAVID]__

### Guidance
* Controller __[JASON]__
* Path Planning __[JASON]__
* Obstacle avoidance

### Simulation
* LIDAR
* Imaging Sonar
* Field components

### GUI Improvements
* RVIZ model spawing __[MATT]__
* Update for new alarms
* Speedometer
* Battery Monitor
* Keyboard driving from camera view
* Incorporate error/alarm display
* Graphs for topics that change with time (rpm, acceleration, etc)

### Refactoring
* Navigation stack purge __[JASON]__
* Modify package structure

### Install Script
* Build and install txros
* Source bashrc after competition
* Rename navigator_ws to mil_ws
* Install git-lfs, tmux
* Download/build velodyne drivers from source
* Figure out dependency caching (mainly ROS) for semaphore

### Networking
* Global control for lab (ip addresses etc)