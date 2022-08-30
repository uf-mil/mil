:mod:`txros` - Independent extensions for ROS
---------------------------------------------

.. automodule:: txros

Starting and Cleaning Up
^^^^^^^^^^^^^^^^^^^^^^^^
When using resources in txros, you must be mindful of how you start the resource,
and how you clean the resource up. Many times, these actions are not done for you
(to give you more control), and you must be mindful of when you plan to do them
yourself. Doing these actions in the improper order or not doing them at all can result
in resources becoming broken.

For example, to start a publisher, you must first call the :meth:`Publisher.setup`
after initializing the class. Then, you must either shut down the publisher using
:meth:`Publisher.shutdown` or shut down the entire node handle using :meth:`NodeHandle.shutdown` - this
will shutdown all publishers spawned by the node.

Furthermore, attempting to shut down a resource more than once will result in an
instance of :class:`RuntimeError` being raised.

Example
^^^^^^^

The following example starts a publisher and a subscriber which listens to the messages
published by the publisher. After two seconds, it reports how many messages it was
able to successfully receive.

.. code-block:: python

    import asyncio
    from rosgraph_msgs.msg import Clock
    from txros import NodeHandle, Publisher, Subscriber
    import uvloop

    async def pubsub(nh: NodeHandle, pub: Publisher, sub: Subscriber):
        count = 0
        try:
            while True:
                await asyncio.sleep(0)
                msg = (Clock(nh.get_time()))
                pub.publish(msg)
                recv = await sub.get_next_message()
                if (recv == msg):
                    count += 1
        except:
            print(f"Eventually, heard {count} messages.")
            pass


    async def main():
        nh = await NodeHandle.from_argv("node", "", anonymous = True)
        async with nh:
            pub = nh.advertise('clock2', Clock, latching = True)
            sub = nh.subscribe('clock2', Clock)
            await asyncio.gather(pub.setup(), sub.setup())
            pubsub_task = asyncio.create_task(pubsub(nh, pub, sub))
            print("Everything is setup. Waiting two seconds.")
            await asyncio.sleep(2)
            pubsub_task.cancel()

    if __name__ == "__main__":
        uvloop.install()
        asyncio.run(main())

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

Exceptions
^^^^^^^^^^
.. attributetable:: txros.XMLRPCException

.. autoclass:: txros.XMLRPCException
    :members:

.. attributetable:: txros.ROSMasterException

.. autoclass:: txros.ROSMasterException
    :members:

.. attributetable:: txros.ServiceError

.. autoclass:: txros.ServiceError
    :members:

.. attributetable:: txros.TooPastError

.. autoclass:: txros.TooPastError
    :members:

Utility Functions
^^^^^^^^^^^^^^^^^

.. autofunction:: txros.wrap_timeout

.. autofunction:: txros.wrap_time_notice

.. autofunction:: txros.wall_sleep

.. autoclass:: XMLRPCLegalType

AsyncServerProxy
^^^^^^^^^^^^^^^^
.. attributetable:: txros.AsyncServerProxy

.. autoclass:: txros.AsyncServerProxy
    :members:

ROSMasterProxy
^^^^^^^^^^^^^^
.. attributetable:: txros.ROSMasterProxy

.. autoclass:: txros.ROSMasterProxy
    :members:

Goal
^^^^
.. attributetable:: txros.Goal

.. autoclass:: txros.Goal
    :members:

GoalManager
^^^^^^^^^^^
.. attributetable:: txros.GoalManager

.. autoclass:: txros.GoalManager
    :members:

SimpleActionServer
^^^^^^^^^^^^^^^^^^
.. attributetable:: txros.SimpleActionServer

.. autoclass:: txros.SimpleActionServer
    :members:

ActionClient
^^^^^^^^^^^^
.. attributetable:: txros.ActionClient

.. autoclass:: txros.ActionClient
    :members:

NodeHandle
^^^^^^^^^^
.. attributetable:: txros.NodeHandle

.. autoclass:: txros.NodeHandle
    :members:

Publisher
^^^^^^^^^
.. attributetable:: txros.Publisher

.. autoclass:: txros.Publisher
    :members:

ServiceClient
^^^^^^^^^^^^^
.. attributetable:: txros.ServiceClient

.. autoclass:: txros.ServiceClient
    :members:

Service
^^^^^^^
.. attributetable:: txros.Service

.. autoclass:: txros.Service
    :members:

Subscriber
^^^^^^^^^^
.. attributetable:: txros.Subscriber

.. autoclass:: txros.Subscriber
    :members:

Transform
^^^^^^^^^^
.. attributetable:: txros.Transform

.. autoclass:: txros.Transform
    :members:

TransformBroadcaster
^^^^^^^^^^^^^^^^^^^^
.. attributetable:: txros.TransformBroadcaster

.. autoclass:: txros.TransformBroadcaster
    :members:

TransformListener
^^^^^^^^^^^^^^^^^
.. attributetable:: txros.TransformListener

.. autoclass:: txros.TransformListener
    :members:
