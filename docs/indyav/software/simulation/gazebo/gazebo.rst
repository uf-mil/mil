IndyAV Gazebo Simulation
========================


How to Use
----------

Launch Gazebo for IndyAV
************************
``roslaunch indyav_launch gazebo.launch``

Launch RVIZ for IndyAV
**********************
in a new pannel
``indyviz``
(this represents what the robot knows)

*NOTE: this will not if run in the docker container*

Launch the gazebo client (visualizer and gui)
*********************************************
in a new pannel
``gazebogui``
(this represents ground truth)

*NOTE: this will not if run in the docker container*

*NOTE: to kill the gazebo client after you are done, usually* ``Ctrl + \`` *isrequired*

*NOTE:* ``gazebogui`` *is a custom alias we have setup to launch gazebo client with some parameters*

Command a Steering angle
************************
in a new pannel::

  rostopic pub /steering indyav_control/SteeringStamped -r 10 '{header: {
  seq: 0,
  stamp: {
    secs: 0,
    nsecs: 0},
  frame_id: ''},
  steering_angle: 0.5}'

Command throttle
****************
in a new pannel::

  rostopic pub /throttle indyav_control/RevsStamped -r 10 '{header: {
  seq: 0,
  stamp: {
    secs: 0,
    nsecs: 0},
  frame_id: ''},
  radians_per_second: 30.000}'

and see the car drive in circles in both Rviz and gazebo client

But How Does Any of This Work
-----------------------------

.. graphviz:: gazebo.dot




