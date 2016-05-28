# Hydrophone Driver

This script allows high level ROS code to interface with the hydrophone board.

### How to run and use
To start the driver, run:

    roslaunch sub8_hydrophone_driver hydrophone_driver.launch

*Note: Make sure the port and baud in /launch/sub8_hydrophone_driver.launch are set correctly*

In order to ask for hydrophone information:

    rosservice call /hydrophone_driver/get_hydrophones

*Note: You may have to press tab twice at the end to get a response, since I haven't been able to test it.*
The service should respond with the timestamp the measurement was taken and 4 floats coorisponding to the 4 hydrophones.
