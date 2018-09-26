# Actuator Board Driver
Driver for interacting with the pneumatic acutator board developed by Daniel Dugger.
Provides a ROS service for opening, closing, or pulseing a solenoid.

### Actuators:

Currently only the following actuators are implemented:
* shooter - `Pulse` Valve
* gripper - `Set` Valve
* dropper - `Pulse` Valve

To add new actuators edit `valves.yaml` (see below).

# How to setup
Setup is done in the `valves.yaml` in the root directory of the package.
Each entry contains information about an actuator. Here are two examples:
    
	shooter:
	  type: 'pulse'
	  ports:
	    open_port:
	      id: 2
	      default: 0
	    close_port:
	      id: -1
	      default: 0
	  pulse_time: 1

	dropper:
	  type: 'set'
	  ports:
	    open_port:
	      id: 3
	      default: 0
	    close_port:
	      id: 4
	      default: 1

The first actuator here is called `shooter`. It is of type `pulse`. 
There are two actuator types: `pulse` and `set`. A `pulse` actuator will toggle breifly before returning back to the inital state (useful for the torpedo for example). A `set` actuator is toggleable between two states.

For `pulse` actuators, you must specify two parameters:
* `ports`: Both the port connected to the actuator that should be enabled to open the device and the port connected to the actuator that should be enabled to close the device. (If one of the ports is not necessary, set it's port id to -1).
* `pulse_time`: This specifies the pulse width (in seconds).

For `set` actuators, you must specify only one parameter:
* `ports`: Both the port connected to the actuator that should be enabled to open the gripper and the port connected to the actuator that should be enabled to close the gripper.

Each listed `port` should contain:
* `id`: The port number on the board. -1 if not a needed port.
* `default`: `1` if the port should start opened, `0` if the port should start closed.

It is important to perseve the structure of each yaml entry, so you can use a previous entry as a template when creating a new entry.

# How to run
To start the driver, run:

    roslaunch sub8_actuator_driver actuator_driver.launch
  
The script should set each of the valves we are using to their default position.

In order to control a given valve run the following code in a new terminal window:

    rosservice call /actuator_driver/actuate "actuator: 'NAME' opened: true" 

Replacing `NAME` with the desired actuator from the list above. For `Set` valves, set opened to `true` or `false` to open or close the valve. For `Pulse` valves calling the service will pulse the valve.

For troubleshooting there is a rosservice called /actuator_driver/actuate_raw that can be called in the same fashion as above but with the `port_id` instead of `NAME`.
