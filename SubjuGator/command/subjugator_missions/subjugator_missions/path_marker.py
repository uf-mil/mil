#! /usr/bin/env python3

from std_msgs.msg import Float32

from .sub_singleton import SubjuGatorMission

SPEED_LIMIT_YAW = 0.2
SPEED_LIMIT = 0.5


class FollowPathMarker(SubjuGatorMission):
    async def run(self, args):
        # Setup angle offset topic
        self.angle_offset = self.nh.subscribe(
            "/angle_offset",
            Float32,
        )

        await self.angle_offset.setup()

        # Follow path marker
        await self.follow_marker()

        # Step forward at the end of the path marker
        await self.step_forward()

        print("Done!")

    async def follow_marker(self):

        while True:

            angle_offset = await self.angle_offset.get_next_message()
            angle_offset = angle_offset.data

            if angle_offset == -999:
                break

            if angle_offset != 0:
                print("Angle to turn:", angle_offset)
                await self.go(
                    self.move().yaw_right_deg(angle_offset).zero_roll_and_pitch(),
                    speed=SPEED_LIMIT_YAW,
                )
            else:
                await self.go(
                    self.move().forward(0.1).zero_roll_and_pitch(),
                    speed=SPEED_LIMIT,
                )

        print("Finished following marker!")

    async def step_forward(self):

        await self.go(self.move().forward(0.5).zero_roll_and_pitch(), speed=SPEED_LIMIT)
