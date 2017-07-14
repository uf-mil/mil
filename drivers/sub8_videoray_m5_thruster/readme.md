VideoRay M5 Thruster Driver
===========================

# Purpose

This node handles the interface between the sub's control computer and the thrusters. It communicates via serial to each thruster, individually issuing commands, it also receives response packets from each of the thrusters (including current, voltage, rpm, faults, etc.) and publishes them.

# Calibration
The thrusters take an input on [-1, 1]. Each thruster can be independently calibrated by editing the forward and backward terms of each thruster in the thruster layout yaml in sub8_thruster_mapper. Each of those is a set of four coefficients for a fourth order polynomial that transforms thrust to effort.

# ROS
The thruster driver takes commands over the topic `thrusters/thrust` and publishes status over `thrusters/thruster_status/{thruster_name}`. A filtered bus_voltage measurement is published over `/bus_voltage`.

Thrusters can be activated and deactivated in software by calling the `/fail_thruster` and `/unfail_thruster` services. The driver will also automatically loss and re-establishment of communications with thrusters and deactivate them accordingly.

