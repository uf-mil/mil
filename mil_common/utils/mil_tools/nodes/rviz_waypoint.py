#!/usr/bin/env python3
from __future__ import division

import asyncio

import txros
import uvloop
from geometry_msgs.msg import PointStamped, PoseStamped
from twisted.internet import defer


class RvizRepublisher:
    async def init(self):
        self.nh = txros.NodeHandle.from_argv("rviz_republisher")
        await self.nh.setup()
        self.point_republish = self.nh.advertise("/rviz_point", PointStamped)
        self.pose_republish = self.nh.advertise("/rviz_goal", PoseStamped)

        self.rviz_goal = self.nh.subscribe("/move_base_simple/goal", PoseStamped)
        self.clicked_point = self.nh.subscribe("/clicked_point", PointStamped)

        await asyncio.gather(
            self.point_republish.setup(),
            self.pose_republish.setup(),
            self.rviz_goal.setup(),
            self.clicked_point.setup(),
        )

        self.delay = 0.1  # s
        await self.publish_point()
        await self.publish_pose()

    async def publish_point(self):
        await self.clicked_point.get_next_message()

        while True:
            await self.nh.sleep(self.delay)
            last_point = self.clicked_point.get_last_message()
            self.point_republish.publish(last_point)

    async def publish_pose(self):
        await self.rviz_goal.get_next_message()

        while True:
            await self.nh.sleep(self.delay)
            last_pose = self.rviz_goal.get_last_message()
            self.pose_republish.publish(last_pose)


async def main():
    rr = RvizRepublisher()
    await rr.init()
    await asyncio.Future()
    await rr.nh.shutdown()


if __name__ == "__main__":
    uvloop.install()
    asyncio.run(main())
