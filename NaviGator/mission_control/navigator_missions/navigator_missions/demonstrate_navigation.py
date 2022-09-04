#!/usr/bin/env python3
import numpy as np
import txros
from mil_misc_tools import ThrowingArgumentParser
from mil_tools import rosmsg_to_numpy
from twisted.internet import defer
from std_srvs.srv import SetBoolRequest

from .navigator import Navigator


class DemonstrateNavigation(Navigator):
    """
    Mission for the "Demonstrate Navigation And Control" challenge.
    May either use the objects in PCODAR or to clicked points.
    TODO: check that moves were completed successfully
    """

    START_MARGIN_METERS = 4.0
    END_MARGIN_METERS = 5.0

    @classmethod
    def init(cls):
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

    @txros.util.cancellableInlineCallbacks
    def run(self, parameters):
        # Go to autonomous mode
        yield self.set_classifier_enabled.wait_for_service()
        yield self.set_classifier_enabled(SetBoolRequest(data=True))
        yield self.nh.sleep(4)
        yield self.change_wrench("autonomous")
        if not parameters.pcodar:
            self.send_feedback(
                "Please click between the end tower of the navigation pass."
            )
            target_point = yield self.rviz_point.get_next_message()
            target_point = rosmsg_to_numpy(target_point.point)
            us = (yield self.tx_pose)[0]
            distance = np.linalg.norm(target_point - us) + self.END_MARGIN_METERS
            distance_per_move = distance / parameters.num_moves
            for i in range(parameters.num_moves):
                self.send_feedback(f"Doing move {i + 1}/{parameters.num_moves}")
                yield self.move.look_at(target_point).forward(distance_per_move).go(
                    blind=True
                )
            defer.returnValue(True)
        else:
            _, closest_reds = yield self.get_sorted_objects("mb_marker_buoy_red", 2)
            _, closest_greens = yield self.get_sorted_objects("mb_marker_buoy_green", 2)

            # Rename the totems for their symantic name
            green_close = closest_greens[0]
            green_far = closest_greens[1]
            red_close = closest_reds[0]
            red_far = closest_reds[1]

            # Get the two center points between gata markers
            begin_midpoint = (green_close + red_close) / 2.0
            end_midpoint = (green_far + red_far) / 2.0

            # Start a little behind the entrance
            yield self.move.set_position(begin_midpoint).look_at(end_midpoint).backward(
                self.START_MARGIN_METERS
            ).go()
            # Then move a little passed the exit
            yield self.move.look_at(end_midpoint).set_position(end_midpoint).forward(
                self.END_MARGIN_METERS
            ).go(blind=True)
            defer.returnValue(True)
