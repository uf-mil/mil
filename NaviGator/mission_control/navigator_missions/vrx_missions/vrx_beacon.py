#!/usr/bin/env python3
import numpy as np
import txros
from mil_tools import rosmsg_to_numpy
from navigator_msgs.srv import AcousticBeaconRequest
from twisted.internet import defer

from .vrx import Vrx

___author___ = "Alex Perez"


class VrxBeacon(Vrx):
    def __init__(self, *args, **kwargs):
        super().__init__(*args, **kwargs)

    async def run(self, parameters):
        self.send_feedback("Waiting for task to start")
        await self.wait_for_task_such_that(
            lambda task: task.state in ["ready", "running"]
        )

        await self.wait_for_task_such_that(lambda task: task.state in ["running"])

        beacon_msg = await self.beacon_landmark(AcousticBeaconRequest())
        print(beacon_msg)

        position = [
            beacon_msg.beacon_position.x,
            beacon_msg.beacon_position.y,
            beacon_msg.beacon_position.z,
        ]

        self.send_feedback(f"Going to {position}")

        goal_pose = [position, [0, 0, 0, 1]]
        await self.move.set_position(goal_pose[0]).set_orientation(goal_pose[1]).go()
