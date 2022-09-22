Examples
^^^^^^^^

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

The node handle can be used for general purposes, such as parameters and sleeping:

.. code-block:: python

    >>> import os
    >>> import txros
    >>> nh = await NodeHandle("/test", "special_node", "localhost", os.environ["ROS_MASTER_URI"], {})
    >>> nh.get_name()
    '/test/special_node'
    >>> await nh.set_param("special_param", True)
    >>> await nh.get_param("special_param")
    True
    >>> await nh.delete_param("special_param")
    >>> try:
    ...     await nh.get_param("special_param")
    ... except txros.ROSMasterException:
    ...     print("This parameter does not exist!")
    This parameter does not exist!
    >>> await nh.sleep(2) # Sleeps for 2 seconds

The node handle can also be used for publishing and subscribing to topics. Note
that all publishers and subscribers must be setup.

.. code-block:: python

    >>> import os
    >>> import txros
    >>> nh = await txros.NodeHandle("/test", "special_node", "localhost", os.environ["ROS_MASTER_URI"], {})
    >>> from std_msgs.msg import Int32
    >>> pub = nh.advertise("running_time", Int32)
    >>> await pub.setup()
    >>> async def publish():
    ...     try:
    ...         count = 0
    ...         while True:
    ...             pub.publish(Int32(count))
    ...             count += 1
    ...             await asyncio.sleep(1)
    ...     except asyncio.CancelledError as _:
    ...         # When task gets cancelled, stop publishing
    ...         pass
    >>> task = asyncio.create_task(publish()) # Start publishing!
    >>> sub = nh.subscribe("running_time", Int32)
    >>> await sub.setup()
    >>> while True:
    ...     print(await sub.get_next_message())
    4
    5
    6
