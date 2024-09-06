# Preflight- Autonomous Robot Verification
## How to Use
Simply type "preflight" anywhere in the MIL directory. Make sure that a robot is connected and running or gazebo is running. To edit the hardware tests list and the automated software tests, edit the tests.py file in mil/mil_common/utils/mil_tools/scripts/mil-preflight/tests.py
## Description
Preflight is an automated testing tool that should be run after turning on and connecting to the robot to run a prelaunch hardware checklist and automated software checklist.

### There are three types of automated software tests
#### Actuators
This test will prompt the user to enable physical moving actuators on the robot. Make sure that the area is cleared and the robot won't damage itself or others nearby. The user will have to watch and validate that they move as expected personally.

To add an actuator test, add a topic with a list of commands and import the module it comes from. See the thrusters example for reference in tests.py
#### Nodes
A ROS Node is a live process currently running and performing a task. They communicate with each other through topics, services, and messages, etc. This test will ensure all listed ROS Nodes are running and alive as expected.

To add a Node test, add a node to the list in tests.py
#### Topics
ROS Topics act as a channel for ROS Nodes to communicate by publishing and subscribing to messages. These tests check to verify that data is being published to these topics, ensuring that sensors under the listed topics are properly reading publishing data.

To add a Topic test, add a topic to the list in tests.py

### Setup Tests
There are also setup tests. These are used to verify certain features on the robot that cannot be automated. For example ensuring that the O-rings are greased.

To add a Setup test, add a what need to be tested to the list in tests.py
