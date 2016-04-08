# Actuator Board Driver

This script allows high level ROS code to interface with the pneumatics board and control actuators.

### Actuators:

Currently only the following actuators are implemented:
* grabber - `Set` Valve
* dropper - `Pulse` Valve
* shooter - `Pulse` Valve

To add new actuators edit `valves.yaml`. Refer to prior actuator implementations in the yaml to add new ones.

# How to run
To start the driver, run:

    roslaunch sub8_actuator_driver actuator_driver.launch
  
The script should set each of the valves we are using to their default position.

In order to control a given valve run the following code in a new terminal window:

    rosservice call /actuator_driver/actuate "actuator: 'NAME' opened: true" 

Replacing `NAME` with the desired actuator from the list above. For `Set` valves, set opened to `true` or `false` to open or close the valve. For `Pulse` valves calling the service will pulse the valve.