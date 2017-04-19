# Sub8 Diagnostics

This package is meant to house diagnostic utilities for SubjuGator

If you add a utility, make sure to add usage instructions below.

## SelfCheck

To do a pre-station-keeping all-systems check:

```
rosrun sub8_diagnostics self_check.py
```

## ThrusterSpinner

To manually spin sets of thrusters or individual thrusters via the keyboard,
or to find out what motor_id's are found on each port:

```
rosrun sub8_diagnostics thruster_spinner.py
```

## ActuatorTester

To manually try closing and opening pneumatic manifold valves or pinging
the actuator board:

```
rosrun sub8_diagnostics actuator_tester.py
```

TODO: make utility to automatically check for thruster out of the water by fully loading
all thrusters and looking at for abnormal current draw
