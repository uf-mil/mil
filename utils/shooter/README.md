#Install/Build
* Install rosserial, rosserial-python and rosserial-arduino `sudo apt-get ros-indigo-rosserial install ros-indigo-rosserial-python ros-indigo-rosserial-arduino`
* clone project into catkin workspace/src
* cd back to catkin workspace
* To build arduino files run `catkin_make navigator_shooter_firmware_arduino`
* To upload the sketch to the arduino run `catkin_make navigator_shooter_firmware_arduino-upload`

#Running / Testing
* Start a ros master node `roscore`
* Start a node to communicate with arduino and turn it into ros stuff `rosrun rosserial_python serial_node.py /dev/USBPATH` ex: `rosrun rosserial_python serial_node.py /dev/ttyACM0`
* Use `rostopic list` to see if the subscribers and publishers you created show up
* Run the command line testing script `rosrun navigator_shooter shooter_control.py`
* Type in one of the valid commands and press enter to test it 

#Messages
* The arduino listens on the topic `/shooter/control` for `std_msgs/String` messages
* The following strings will result in behavior on the shooter:

Command | Behavior 
--- | --- |
shoot | Shoots all 4 balls using timing
cancel | cancels any command, turning off all motors
flyon | manually turns on the fly wheel motors
flyoff | manually turns off the fly wheel motors
feedon | manually turns on the feeder motor
feedoff | manually turns off the feeder motor
ledon | turns on the built in LED on pin 13 of the arduino for testing
ledon | turns off the built in LED on pin 13 of the arduino for testing




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


