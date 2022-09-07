import asyncio

from geometry_msgs.msg import Point, PointStamped
from txros import NodeHandle


async def main():
    nh = NodeHandle.from_argv("my_special_node")
    await nh.setup()
    pub = nh.advertise("special_point", Point)
    pub.publish(PointStamped())  # This is an error
    await nh.shutdown()


asyncio.run(main())
