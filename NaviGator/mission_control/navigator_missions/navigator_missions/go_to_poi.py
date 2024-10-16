#!/usr/bin/env python3
import math

import numpy as np
from geometry_msgs.msg import Point, Pose, Quaternion
from tf.transformations import euler_from_quaternion, quaternion_from_euler

from .navigator import NaviGatorMission


class GoToPOI(NaviGatorMission):
    """
    Moves NaviGator to a point of interest
    """

    @classmethod
    def decode_parameters(cls, parameters):
        parameters = parameters.split()
        if len(parameters) != 1:
            raise Exception("Only one parameter accepted")
        return parameters[0]

    async def run(self, poi):
        self.send_feedback("Waiting for " + poi)
        position, orientation = await self.poi.get(poi)
        orientation[0:2] = np.zeros(2)
        euler = euler_from_quaternion(orientation)
        euler += np.array([0, 0, math.pi])
        orientation = quaternion_from_euler(*euler)
        self.send_feedback(
            f"Moving to {poi} at {position[0:2]} with orientation {orientation}",
        )
        await self.change_wrench("autonomous")
        await self.move.to_pose(Pose(Point(*position), Quaternion(*orientation))).go(blind=True)
        return "Success"
