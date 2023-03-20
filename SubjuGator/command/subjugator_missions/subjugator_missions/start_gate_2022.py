#!/usr/bin/env python

import math

from axros import Subscriber
from mil_misc_tools import text_effects
from sensor_msgs.msg import MagneticField
from tf.transformations import *

from .sub_singleton import SubjuGatorMission

fprint = text_effects.FprintFactory(title="START_GATE", msg_color="cyan").fprint

SPEED = 0.6

CAREFUL_SPEED = 0.3

# How many meters to pass the gate by
DIST_AFTER_GATE = 1
WAIT_SECONDS = 1

RIGHT_OR_LEFT = 1


class StartGate2022(SubjuGatorMission):
    async def current_angle(self):
        imu_sub: Subscriber[MagneticField] = self.nh.subscribe(
            "/imu/mag",
            MagneticField,
        )
        async with imu_sub:
            reading = await imu_sub.get_next_message()
        declination = await self.nh.get_param("/course/location/declination")
        assert isinstance(declination, (float, int))
        return (
            math.atan2(reading.magnetic_field.z, reading.magnetic_field.y)
            * 180
            / math.pi
            + declination
        )

    async def run(self, args):
        fprint("Waiting for odom")
        await self.tx_pose()

        fprint(f"Waiting {WAIT_SECONDS} seconds...")
        await self.nh.sleep(WAIT_SECONDS)

        orientation = await self.nh.get_param("/course/start_gate/orientation")
        fprint(f"Rotating to start gate orientation (of {orientation} degrees)...")
        cur_degree = await self.current_angle()
        fprint(f"Current degree: {cur_degree} degrees")
        assert isinstance(orientation, int)
        if cur_degree > orientation:
            fprint(f"Yaw lefting: {cur_degree - orientation} degrees")
            await self.go(self.move().yaw_left_deg(cur_degree - orientation))
        else:
            fprint(f"Yaw righting: {orientation - cur_degree} degrees")
            await self.go(self.move().yaw_right_deg(orientation - cur_degree))
        await self.nh.sleep(2)  # Give sub time to turn before moving straight

        fprint("Found odom")
        down = self.move().down(1)
        await self.go(down, speed=SPEED)

        forward = self.move().forward(4)
        await self.go(forward, speed=SPEED)
