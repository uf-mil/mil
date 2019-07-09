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

```sh
thruster_spinner
#or
rosrun sub8_diagnostics thruster_spinner.py
```

## ThrusterMonitor

The thruster monitor is a special tmux session for monitoring thruster statuses, wrenches, bus voltage, and other things. Users don't need to launch it, as that is done by the 'thrusters' launch file.

To attach to it, use the alias `thruster_monitor`.

## ThrusterDebugShell

To manually send commands to thrusters, among many other ways of interacting with them using ThrusterPort functionality:

```sh
thruster_debug_shell
#or
rosrun sub8_diagnostics thruster_debug_shell.py
```

## ActuatorTester

To manually try closing and opening pneumatic manifold valves or pinging
the actuator board:

```
rosrun sub8_diagnostics actuator_tester.py
```

TODO: make utility to automatically check for thruster out of the water by fully loading
all thrusters and looking at for abnormal current draw
