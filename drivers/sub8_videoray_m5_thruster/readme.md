VideoRay M5 Thruster Driver
===========================

# Authors

This code was originally authored by Forrest Voight (forrestv) and Matthew Griessler.
It is now maintained by David Soto.

# Purpose

This node handles the interface between the sub's control computer and the thrusters. It communicates via serial to each thruster, individually issuing commands, it also receives response packets from each of the thrusters (including current, voltage, rpm, faults, etc.) and publishes them.

# Calibration
The thrusters take an input on [-1, 1]. The calibration.json file provided maps commanded effort values to produced output thrust in Newtons.

Run with the argument `--calibration_path=$(find sub8_videoray_m5_thruster)/config/calibration.json`, or the path to the appropriate calibration file


# ROS
The thruster driver takes commands over the topic `thrusters/thrust` and publishes status over `thrusters/thruster_status`
