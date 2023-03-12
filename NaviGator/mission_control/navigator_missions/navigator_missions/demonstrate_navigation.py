#!/usr/bin/env python3
import numpy as np
from mil_misc_tools import ThrowingArgumentParser
from mil_tools import rosmsg_to_numpy
from std_srvs.srv import SetBoolRequest

from .navigator import NaviGatorMission


class DemonstrateNavigation(NaviGatorMission):
    """
    Mission for the "Demonstrate Navigation And Control" challenge.
    May either use the objects in PCODAR or to clicked points.
    TODO: check that moves were completed successfully
    """

    START_MARGIN_METERS = 4.0
    END_MARGIN_METERS = 5.0

    @classmethod
    async def setup(cls):
        parser = ThrowingArgumentParser(description="Navigation Pass Mission")
        parser.add_argument(
            "--use-pcodar",
            dest="pcodar",
            action="store_true",
            help="User PCODAR objects instead of clicked point",
        )
        parser.add_argument(
            "-m",
            "--moves",
            dest="num_moves",
            type=int,
            default=5,
            help="Number of moves to make, more is more likely to not leave gate",
        )
        cls.parser = parser

    @classmethod
    def decode_parameters(cls, parameters):
        argv = parameters.split()
        return cls.parser.parse_args(argv)

    async def run(self, parameters):
        # Go to autonomous mode
        await self.set_classifier_enabled.wait_for_service()
        await self.set_classifier_enabled(SetBoolRequest(data=True))
        await self.change_wrench("autonomous")
        if not parameters.pcodar:
            print(1)
            self.send_feedback(
                "Please click between the end tower of the navigation pass.",
            )
            target_point = await self.rviz_point.get_next_message()
            target_point = rosmsg_to_numpy(target_point.point)
            us = (await self.tx_pose())[0]
            us = (await self.tx_pose())[0]
            distance = np.linalg.norm(target_point - us) + self.END_MARGIN_METERS
            distance_per_move = distance / parameters.num_moves
            for i in range(parameters.num_moves):
                self.send_feedback(f"Doing move {i + 1}/{parameters.num_moves}")
                await self.move.look_at(target_point).forward(distance_per_move).go(
                    blind=True,
                )
            return True
        else:
            await self.nh.sleep(3)
            _, closest_reds = await self.get_sorted_objects("red_cylinder", 1)
            _, closest_greens = await self.get_sorted_objects("green_cylinder", 1)

            # Rename the totems for their symantic name
            green_close = closest_greens[0]
            red_close = closest_reds[0]

            # Get the two center points between gata markers
            begin_midpoint = (green_close + red_close) / 2.0

            # Start a little behind the entrance
            await self.move.set_position(begin_midpoint).backward(
                self.START_MARGIN_METERS,
            ).go()

            _, closest_reds = await self.get_sorted_objects("red_cylinder", 2)
            _, closest_greens = await self.get_sorted_objects("green_cylinder", 2)

            # Rename the totems for their symantic name
            green_far = closest_greens[1]
            red_far = closest_reds[1]

            # Get the two center points between gata markers
            end_midpoint = (green_far + red_far) / 2.0

            # Then move a little passed the exit
            await self.move.look_at(end_midpoint).set_position(end_midpoint).forward(
                self.END_MARGIN_METERS,
            ).go()
            print("GO NAVIGATOR")
            return True
