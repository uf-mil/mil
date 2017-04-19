# sub8_diagnostics

This package is meant to house diagnostic utilities for SubjuGator

If you add a utility, make sure to add usage instructions below.

## ActuatorTester

To manually try closing and opening pneumatic manifold valves or pinging
the actuator board:

'''
rosrun sub8_diagnostics actuator_tester.py
'''

## ThrusterSpinner

To manually spin sets of thrusters or individual thrusters via the keyboard,
or to find out what motor_id's are found on each port:

'''
rosrun sub8_diagnostics thruster_spinner.py
'''

