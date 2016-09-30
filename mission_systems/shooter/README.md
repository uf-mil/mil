#Install/Build
* Install rosserial, rosserial-python and rosserial-arduino `sudo apt-get ros-indigo-rosserial install ros-indigo-rosserial-python ros-indigo-rosserial-arduino`
* clone project into catkin workspace/src
* cd back to catkin workspace
* To build arduino files run `catkin_make navigator_shooter_firmware_arduino`
* To upload the sketch to the arduino run `catkin_make navigator_shooter_firmware_arduino-upload`

#Running / Testing
* Start a ros master node `roscore`
* Launch the rosserial node for the arduino `roslaunch navigator_launch hardware_drivers.launch`

#Config
The timing values (in milliseconds) for the load and fire commands are configurable from launch/config.yaml

#Control
The arduino firmware provides the following services for controlling the shooter:

| Service | Type | Behavior | Arguments/Returns |
| ------- | ---- | -------- | --------- |
| /shooter/cancel | std_srvs/Trigger | Immediately turns off everything on the shooter | None |
| /shooter/load | std_srvs/Trigger | retracts the feeder to allow a ball in, pushes the fall towards the fly wheels, then turns on the flywheels for quickfiring | returns success == false if ball already loaded or shooter is currently doing something |
| /shooter/fire | std_srvs/Trigger | extends the linear actuator to launch the ball, then turns off the flywheels | None | returns success == false if ball is NOT loaded or shooter is currently doing something |
| /shooter/manual | navigator_msgs/ShooterManual | Manually control the motor controllers on the shooter| feeder: int32 (-100 to 100 speed to set feeder), shooter: int32  (-100 to 100 speed to set flywheels) |

#Switching USB/Board type
* open firmware/CMakeLists.txt
* To change board, edit text after BOARD within generate_arduino_firmware function
* To change USB port path, edit text after PORT within generate_arduino_firmware function
ex:
```
generate_arduino_firmware(arduino
  SRCS shooter.cpp ${ROS_LIB_DIR}/time.cpp
  BOARD uno
  PORT /dev/ttyACM0
)
```


