#! /usr/bin/env python3
from .sub_singleton import SubjuGator

SIDE_LENGTH = 1  # meters
SPEED_LIMIT = 0.2  # m/s


class Square(SubjuGator):
    async def run(self, args):
        center = self.move.forward(0).zero_roll_and_pitch()
        for i in range(4):
            forward = self.move.forward(SIDE_LENGTH).zero_roll_and_pitch()
            right = forward.right(SIDE_LENGTH).zero_roll_and_pitch()

            await forward.go(speed=SPEED_LIMIT)
            await right.go(speed=SPEED_LIMIT)
            await forward.go(speed=SPEED_LIMIT)
            await center.go(speed=SPEED_LIMIT)
            center = center.yaw_right_deg(90)
            await center.go(speed=SPEED_LIMIT)

        print("Done!")
