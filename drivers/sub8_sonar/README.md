# Sonar Driver

This script interfaces with Jake's sonar board and provides a ROS service that returns the location and time of emission of the last heard pinger pulse.

### How to run and use
To start the driver, run:

    roslaunch sub8_sonar sonar.launch

*Note: In /launch/sonar.launch, make sure that port and baud are set correctly, and that the hydrophones' coordinates in the sonar frame are accurate.*

In order to ask for hydrophone information:

    rosservice call /sonar/get_pinger_pulse

*Note: You may have to press tab twice at the end to get a response, since I haven't been able to test it.*
The service should respond with the x, y, z, and t of the emission of the last heard pinger pulse.
