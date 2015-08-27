## Alarm Structure

An alarm will be described by the following: 

1. **action_required**: bool
1. **problem_description**: string
1. **severity**: int -> 0 is most severe 
1. **problem_flag**: char -> indexes into table of possible system alarms maintained by the alarm node
1. **node_id**: string -> the node where the alarm originated from 

## Alarm Severity 

* 0 - FAILURE (fatal)
* 1 - CRITICAL (Usually fatal) 
* 2 - WARNING (Non-fatal) 
* 3 - INFO 
* 4 - DEBUG 

## Alarm Node

The Sub8 system will have an alarm ROS node that encapsulates how the system will respond to each 
alarm type 

## Component Specific Alarms

> _Alarm description (severity level): response_

### Trajectory Generator

1. Planning failed (2): Will try to use a safety path
1. Switching to safety path (4): Continue
1. Entered unavoidable collision zone (0): abort
1. Starting to re-plan (3): Continue
1. Re-plan success (3): Continue
1. Re-plan failure (2): Will try to use a safety path 
1. No usable safety paths (1): No where to go, abort 