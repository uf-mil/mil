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
