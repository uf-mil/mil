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

## Alarm Node

The Sub8 system will have an alarm ROS node that encapsulates how the system will respond to each 
alarm type 

## Component-Specific Alarms

> _Alarm description (severity level): response_

### Trajectory Generator

1. Entered unavoidable collision zone (0): abort mission
1. No usable safety paths (1): abort mission
1. Planning failed (2): try to use a safety path
1. Re-plan failure (2): try to use a safety path 
