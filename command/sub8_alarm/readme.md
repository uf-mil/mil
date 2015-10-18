Alarms
======

Alarms are the means by which Subjugator 8 responds to disaster. It is the spiritual successor to the kill system of previous iterations.

See the wiki to understand their implementation, how they work, and how to add alarms.


# Adding Alarms

In the "alarms" sub-module: sub8_alarm.alarms, there are *.py files, for addressing particular alarm scenarios.


# Topics

Alarms are published on /alarms, do not publish an alarm directly, always use the  AlarmBroadcaster class to handle most of the things.


# TODO
* Add integration tests
* NOTE: The vehicle behaving correctly when an alarm is called is distinct from the alarm strictly working
    * These tests should fail if the alarm is incorrectly implemented
    * They don't need to (but can) fail if the rest of the vehicle can't handle it properly