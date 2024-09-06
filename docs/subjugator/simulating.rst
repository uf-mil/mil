Simulating SubjuGator
---------------------

Launch Gazebo Server
~~~~~~~~~~~~~~~~~~~~

In one panel, run:

.. code-block:: bash

    $ roslaunch subjugator_launch gazebo.launch

This will launch the server side of the gazebo simulator. The gazebo server (a.k.a. ``gzserver``)
is the half of the simulator that will do all the heavy lifting in terms of simulating physics,
running stock / custom plugins (of which we have many), and tying into ROS.

Run Gazebo Client
~~~~~~~~~~~~~~~~~

Gazebo also has GUI visualization component named Gazebo Client (a.k.a ``gzclient``).
This half of the simulator really only does image rendering of the simulated environment.

.. warning::

    Simulated cameras are rendered in `gzserver`.

We have a custom `alias <https://alvinalexander.com/blog/post/linux-unix/create-aliases>`__ setup
to run the gazebogui with special parameters:

.. code-block:: bash

    $ gazebogui

This will show you ground truth of the simulated environment. However, when debugging robot behavior, it is usually more usful to have an idea of exactly what the robot knows, which is why there is also Rviz.

Run RVIZ
~~~~~~~~

You can visualize what the robot knows by running the sub visualization alias in
a new terminal:

.. code-block:: bash

   $ subviz

``subviz`` is an `alias <https://alvinalexander.com/blog/post/linux-unix/create-aliases>`__
which launches `RVIZ <http://wiki.ros.org/rviz>`__ with a configuration
for SubjuGator. Generally speaking, this will show what information is available
to the robot.

Clear the Kill
~~~~~~~~~~~~~~

Whenever SubjuGator is initialized, it is a state called "kill" for
safety. While in the kill state, the sub will not actuate in any way so
it is safe for people the physically handle it. The sub must be put into
a state of "unkill" to actually do anything. To unkill the sub, go to a
new panel and run:

.. code-block:: bash

    $ amonitor kill

and then hold ``Shift + C`` until the terminal turns green.

To initiate a software kill, press the space bar into the same terminal where
you unkilled.

Give a move command
~~~~~~~~~~~~~~~~~~~

Give SubjuGator a move command with ``submove``:

.. code-block:: bash

   $ submove forward 5m

The sub should move forward by 5 meters.

See the current odometry
~~~~~~~~~~~~~~~~~~~~~~~~
Try streaming the content of a rostopic in a new panel by running:

.. code-block:: bash

    $ rostopic echo /odom
