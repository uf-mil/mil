#**Kill System**

**Below are the details of the kill system for Navigator. It is what we use to kill power to the motors depending on certain circumstances. It is a software kill only. This means it only stops commands from being sent to the motors, it does not cut off power.**

A "kill" refers to a reason why the boat is presently shutdown. For example, when we lose communication we will issue a kill command and it is known as a "communication kill".

###**Methodology**:

The kill system is a node with several key features. It is considered a dormant node and thus uses a ROS service. It is polled when data is sent/requested rather than constantly passing messages. 
* Take in kill and revive signals from several sources
* List the kills currently active in the system.
* Revive all kills at once

###**Initial Necessary Components**
* RC kill that can be triggered from the RC node
* Ability to list that an RC kill is currently happening.