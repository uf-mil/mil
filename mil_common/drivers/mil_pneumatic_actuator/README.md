# Actuator Board Driver
Driver for interacting with the pneumatic acutator board developed by Daniel Dugger.
Provides a ROS service for opening, closing, or pulseing a solenoid.

## Configuration
See example.launch

## Usage/Interfaces
* Start the program `roslaunch mil_pneumatic_actuator example.launch`
* `/actuator_driver/actuate`: service to open/close/pulse an actuator
* `/actuator_driver/reset`: reset all actuators to their default state as defined in parameters
