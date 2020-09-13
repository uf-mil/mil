SubjuGator
==========

This page provides general info about developing for
`RoboSub <https://robonation.org/programs/robosub/>`__, an anual
competition for autonomous submarines. MIL has participated in RoboSub
for 20+ years. Most code for this is hosted under SubjuGator/.

*NOTE: Please go through the* `development guide </docs/development/development_guide>`__ 
*before going through this tutorial. We will assume you are currently in the development container
running a tmux session.*

Software Design
---------------
For an overview of the various nodes running on SubjuGator and how they
interact, see the below graph (open in a new window).

.. graphviz:: ../../SubjuGator/docs/high_level_architecture.dot

Running SubjuGator in Simulation
--------------------------------

Launch Gazebo Server
~~~~~~~~~~~~~~~~~~~~

In one panel, ``roslaunch sub8_launch gazebo.launch`` This
will launch the server side of the gazebo simulator. The gazebo server (a.k.a. gzserver)
is the half of the simulator that will do all the heavy lifting in terms of simulating phyiscs,
running stock / custom plugins (of which we have many), and tying into ROS.

Run Gazebo Client
~~~~~~~~~~~~~~~~~

Gazebo also has gui visulalization componet to it called Gazebo Client (a.k.a gzclient).
This half of the simulator really only does image rendering of the simulated environment.

*NOTE: The Gazebo Client will not work inside of the development docker container*

*NOTE: Simulated cameras are rendered in gzserver*

We have a custom `alias <https://alvinalexander.com/blog/post/linux-unix/create-aliases>`__ setup
to run the gazebogui with special parameters:

``gazebogui``

This will show you ground truth of the simulated environment. However, when debuggin robot behavior, it is usually more usful to have an idea of exactly what the robot knows, which is why there is also Rviz.

Run RVIZ
~~~~~~~~

You can visualize what the robot knows by running ``subviz`` in a new panel.

*NOTE: Rviz will not work inside of the development docker container*

``subviz`` is an
`alias <https://alvinalexander.com/blog/post/linux-unix/create-aliases>`__
which launches `RVIZ <http://wiki.ros.org/rviz>`__ with a configuration
for SubjuGator. Generally speaking, this will show what information is
avalible to the robot.

Clear the Kill
~~~~~~~~~~~~~~

Whenever SubjuGator is initialized, it is a state called "kill" for
saftey. While in the kill state, the sub will not actuate in any way so
it is safe for people the physically handle it. The sub must be put into
a state of "unkill" to actually do anything. To unkill the sub, go to a
new panel and run, ``amonitor kill`` and then hold ``Shift+c`` until the
terminal turns green. To initiate a software kill, press the space bar
into the same terminal where you unkilled.

*NOTE: Hardware sytems can also raise a kill.*

Give a move command
~~~~~~~~~~~~~~~~~~~

Give SubjuGator a move command with ``submove forward 5m`` also in a new
panel.

*NOTE: You can also just type* ``submove f 5``

See the current odometry
~~~~~~~~~~~~~~~~~~~~~~~~

Try streaming the content of a rostopic in a new pannel by running
``rostopic echo /odom``
