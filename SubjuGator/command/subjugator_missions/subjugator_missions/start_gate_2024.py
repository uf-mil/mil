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


class StartGate2024(SubjuGatorMission):
    async def current_angle(self):
        imu_sub: Subscriber[MagneticField] = self.nh.subscribe(
            name="/imu/mag",
            message_type=MagneticField,
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

    async def is_left(self, team) -> bool:
        assert isinstance(team, str)
        return await self.identify_side(team) == "left"

    async def identify_side(self, team: str) -> str:
        """
        Identifies which side the side the desired team is on using computer
        vision.

        Args:
            team (str): Identifies the team to look for. Will either be 'red'
                or 'blue'.

        Returns:
            str: Either 'left' or 'right', depending on which side the team is.
        """
        # TODO: This needs to be implemented using CV or a similar algo.
        return "left"

    async def run(self, args):
        ### Start of mission
        fprint("Waiting for odom")
        await self.tx_pose()

        fprint(f"Waiting {WAIT_SECONDS} seconds...")
        await self.nh.sleep(WAIT_SECONDS)

        ### Rotate to the start gate
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

        ### Move down into the water
        down = self.move().down(1)
        await self.go(down, speed=SPEED)

        ### Choose a side and move to it
        team = await self.nh.get_param("/strategy/team")
        fprint(f"Identifying the side of our team ({team})")
        is_left = await self.is_left(team)

        ### Move to the side of the start gate
        fprint(f"Team was identified. Moving to the {team} side of the gate.")
        if is_left:
            left = self.move().left(1)
            await self.go(left, speed=SPEED)
        else:
            right = self.move().right(1)
            await self.go(right, speed=SPEED)

        ### Move through the start gate
        forward = self.move().forward(7)
        await self.go(forward, speed=SPEED)

        ### Complete style if requested
        style = await self.nh.get_param("/strategy/start_gate/with_style")
        assert isinstance(style, bool)
        if style:
            fprint("Completing style...")
            #### Rotate left
            rotations = [90] * 4
            for rotation in rotations:
                await self.go(self.move().yaw_left_deg(rotation), speed=SPEED)
                await self.nh.sleep(0.5)
            await self.nh.sleep(1)  # Give sub some time to catch up

            #### Rotate right
            rotations = [90] * 4
            for rotation in rotations:
                await self.go(self.move().yaw_right_deg(rotation), speed=SPEED)
                await self.nh.sleep(0.5)

        else:
            fprint("Not completing style (not requested).")

        ### End of mission
