Errors and Known Issues
^^^^^^^^^^^^^^^^^^^^^^^

Handling :class:`aiohttp.ClientConnectionError`
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

When using txros, you may encounter an instance of :class:`aiohttp.ClientConnectionError`
when attempting to use the module. By the class' name, its understood that it is
related to some connection error... but how do you actually fix it?

Check the following things:

* `roscore` is running and accessible
* The correct ROS Master URI is given to the :class:`~txros.NodeHandle` or :class:`~txros.ROSMasterProxy`.

If these appear correct, try restarting ROS (either by restarting `roscore` or `roslaunch`).
It may be the case that a node stopped incorrectly or a service was improperly started,
in which case ROS may believe a resource exists when it actually does not.

Subscribing to parameter updates
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

Using `rospy`, individual nodes can "subscribe" to particular parameters to receive
updates on when these parameters are updated. This allows the nodes to quickly respond
to changes made in the parameter server.

However, this behavior has not been implemented in txros. This is because a more
modern approach to this behavior includes using `Dynamic Reconfigure <https://wiki.ros.org/dynamic_reconfigure>`_.
Dynamic Reconfigure is more efficient than using the parameter subscriber feature
previously mentioned, as dynamic reconfigure uses callbacks to indicate new changes,
while nodes use a consistent polling feature to get updates on parameters.
