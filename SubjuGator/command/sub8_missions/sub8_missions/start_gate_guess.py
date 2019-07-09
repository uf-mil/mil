from txros import util
import numpy as np
import mil_ros_tools
from mil_misc_tools import text_effects
from sub8_msgs.srv import GuessRequest, GuessRequestRequest
from .sub_singleton import SubjuGator

fprint = text_effects.FprintFactory(title="PINGER", msg_color="cyan").fprint

SPEED = 0.2
DOWN_SPEED = 0.1

DOWN = 1.5


class StartGateGuess(SubjuGator):

    @util.cancellableInlineCallbacks
    def run(self, args):

        fprint('Getting Guess Locations')

        sub_start_position, sub_start_orientation = yield self.tx_pose()

        gate_txros = yield self.nh.get_service_client('/guess_location',
                                                      GuessRequest)
        gate_1_req = yield gate_txros(GuessRequestRequest(item='start_gate1'))
        gate_2_req = yield gate_txros(GuessRequestRequest(item='start_gate2'))

        gate_1 = mil_ros_tools.rosmsg_to_numpy(
            gate_1_req.location.pose.position)
        gate_2 = mil_ros_tools.rosmsg_to_numpy(
            gate_2_req.location.pose.position)

        mid = (gate_1 + gate_2) / 2
        fprint('Found mid {}'.format(mid))

        fprint('Looking at gate')
        yield self.move.down(DOWN).set_orientation(sub_start_orientation).go(
            speed=DOWN_SPEED)
        yield self.move.look_at_without_pitching(mid).go(speed=DOWN_SPEED)

        fprint('Going!')
        yield self.move.set_position(mid).depth(DOWN).go(speed=SPEED)
