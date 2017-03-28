# Sonar Driver

This script interfaces with Jake's sonar board and provides a ROS service that returns the location and time of emission of the last heard pinger pulse.

## How to run and use
To start the driver, run:

    rosrun mil_passive_sonar sonar.launch

> Note: make sure that port, and baud rate, and hydrophone locations are loaded into the parameter server. See launch/test.launch for an example.

In order to ask for hydrophone information:

    rosservice call /sonar/get_pinger_pulse *double_tab*

The service should respond with the x, y, and z of the last heard pinger
pulse. Remember that this should be considered the tip of an arrow
pointing in the direction of the pinger, the range is completely unreliable
and varies wildly.

## TODO
+ This package is not yet fully set up to use the paulbaurd. The interface to make this happen would be simple to implement.
+ This package should estimate the least squares solution for the actual 3d position of the pinger.
+ Visualize both individual heading veactors and the LS position estimate.
