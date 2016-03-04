Sub Mission Manager
==================

# Running the sub

    roslaunch sub8_launch sub8.launch

# Running a Mission

    rosrun sub8_missions mission stop

    rosrun sub8_missions mission forward_1_m

    rosrun sub8_missions mission level_off


# Why we're using TXROS

* We want to be able to wait for the next message from a topic without having to loop-and-wait

* We want to be able to do things asynchronously

* We want to be able to externally enforce timeouts

* Waiting for things to be ready is much more intelligent (Than initializing with None and then polling)

# TODO
    - Chaining together multiple missions in the command line
    - Responding to sensor input
    - Intelligently using motion planning (instead of simply using moveto)
    - Allow missions to have more control over failure conditions, etc
    - Allow for time-out conditions (Enforce them, if need-be)