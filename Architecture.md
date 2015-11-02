Architecture Overview
====================

TODO: Update to include navigation nodelet

# Simple Diagram
This diagram is a draft, and is not all-inclusive, but should contain all of the information one needs to begin adding software. Note: It does not contain the alarm system.

![Simple Architecture Diagram](http://i.imgur.com/imkjbCL.png?1)


# Trajectories

* A Trajectory, all of whose waypoints exist in a single (but arbitrary) frame, is published by the Trajectory Generator. The controller then takes that trajectory and achieves those waypoints, and determines internally when a waypoint has been achieved.
    * See sub8_msgs/Trajectory.msg and sub8_msgs/Waypoint.msg

# Considerations
The sub should be able to...
- Change its trajectory based on new information at any time
- Safely respond to an unrecoverable error
- Internally handle typical problems, without needing a restart
- Perform actions while moving
- Perform multiple non-conflicting actions at once
    - (Move while pointing; Point and open gripper; etc)
- Navigate without landmarks safely for some distance
- Generate and operate on a traversability map

# Open Questions
- Can we ever make all three cameras work? Is it a bus bandwidth problem?