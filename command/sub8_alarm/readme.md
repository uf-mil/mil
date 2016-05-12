Alarms
======

Alarms are the means by which Subjugator 8 responds to disaster. It is the spiritual successor to the kill system of previous iterations.

See the wiki to understand their implementation, how they work, and how to add alarms.


# Command-line interface

```shell

# To clear a particular alarm from the CLI
rosrun sub8_alarm clear network-timeout

# Raise a particular alarm
rosrun sub8_alarm raise network-timeout

# Clear all alarms
rosrun sub8_alarm clear all

```

# Adding Alarms

In the "alarms" sub-module: sub8_alarm.alarms, there are *.py files, for addressing particular alarm scenarios.


# Topics

Alarms are published on /alarms, do not publish an alarm directly, always use the  AlarmBroadcaster class to handle most of the things.


# Meta-Alarms

Meta alarms are alarms that are triggered by (potentially) a whole group of alarms. This needs some work.


# TODO
* NOTE: The vehicle behaving correctly when an alarm is called is distinct from the alarm strictly working
    * These tests should fail if the alarm is incorrectly implemented
    * They don't need to (but can) fail if the rest of the vehicle can't handle it properly
