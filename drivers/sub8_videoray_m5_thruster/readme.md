VideoRay M5 Thruster Driver
===========================

# Authors

This code was originally authored by Forrest Voight (forrestv) and Matthew Griessler.

# Purpose

This node handles the interface between the sub's control computer and the thrusters. It communicates via serial to each thruster, individually issuing commands.

# Configuration
The thrusters take an input on [-1, 1]. The configuration.json file provided relates an maps a desired thrust in Newtons to the thruster's acceptable input range.

Run with the argument `--configuration_path=$(find sub8_videoray_m5_thruster)/config/calibration.json`, or the path to the appropriate calibration file


# ROS
