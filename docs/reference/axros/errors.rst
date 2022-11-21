Errors and Known Issues
^^^^^^^^^^^^^^^^^^^^^^^

Handling :class:`aiohttp.ClientConnectionError`
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

When using axros, you may encounter an instance of :class:`aiohttp.ClientConnectionError`
when attempting to use the module. By the class' name, its understood that it is
related to some connection error... but how do you actually fix it?

Check the following things:

* `roscore` is running and accessible
* The correct ROS Master URI is given to the :class:`~axros.NodeHandle` or :class:`~axros.ROSMasterProxy`.

If these appear correct, try restarting ROS (either by restarting `roscore` or `roslaunch`).
It may be the case that a node stopped incorrectly or a service was improperly started,
in which case ROS may believe a resource exists when it actually does not.

Subscribing to parameter updates
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

Using `rospy`, individual nodes can "subscribe" to particular parameters to receive
updates on when these parameters are updated. This allows the nodes to quickly respond
to changes made in the parameter server.

However, this behavior has not been implemented in axros. This is because a more
modern approach to this behavior includes using `Dynamic Reconfigure <https://wiki.ros.org/dynamic_reconfigure>`_.
Dynamic Reconfigure is more efficient than using the parameter subscriber feature
previously mentioned, as dynamic reconfigure uses callbacks to indicate new changes,
while nodes use a consistent polling feature to get updates on parameters.

Subscribing to high-rate publishers
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

Some publishers publish messages at very fast rates, which can add reliability
and speed to a robotic system. However, axros may not be able to keep up with the
high rate of incoming messages if the client code is constructed in particular
ways.

To ensure that the subscriber is able to read all incoming messages, ensure the following:

1. Supply a callback to the subscriber that receives all incoming messages; do
   not depend on using :meth:`axros.Subscriber.get_next_message`. The latter method
   will cause messages to never be read if a callback is not supplied.
2. When attempting to read all messages published by a publisher over a specific
   period, use :meth:`axros.Subscriber.recently_read` to determine if more messages
   still need to be read before the subscriber can be closed.
