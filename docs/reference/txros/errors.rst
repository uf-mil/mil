Common Errors
^^^^^^^^^^^^^

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
