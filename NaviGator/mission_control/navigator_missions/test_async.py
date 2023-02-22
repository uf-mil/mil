import asyncio

from axros import NodeHandle
from geometry_msgs.msg import Point, PointStamped


async def main():
    nh = NodeHandle.from_argv("my_special_node")
    await nh.setup()
    pub = nh.advertise("special_point", Point)
    pub.publish(PointStamped())  # This is an error
    await nh.shutdown()


asyncio.run(main())
