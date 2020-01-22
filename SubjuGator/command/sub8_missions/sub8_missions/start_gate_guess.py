from txros import util
import rospy
from std_srvs.srv import Trigger
import numpy as np
import mil_ros_tools
from mil_misc_tools import text_effects
from sub8_msgs.srv import GuessRequest, GuessRequestRequest
from .sub_singleton import SubjuGator

fprint = text_effects.FprintFactory(title="PINGER", msg_color="cyan").fprint

SPEED = 1
DOWN_SPEED = 0.5

DEPTH = 0.5


class StartGateGuess(SubjuGator):
    @util.cancellableInlineCallbacks
    def run(self, args):

      fprint('Getting Guess Locations')

      sub_start_position, sub_start_orientation = yield self.tx_pose()

      gate_1 = yield self.poi.get('start_gate1')
      gate_2 = yield self.poi.get('start_gate2')

      mid = (gate_1 + gate_2) / 2
      norm = sub_start_position - mid
      norm = norm / np.linalg.norm(norm)
      fprint('Found mid {}'.format(mid))

      fprint('Looking at gate')
      yield self.move.depth(DEPTH).go(speed=DOWN_SPEED)
      yield self.move.look_at_without_pitching(mid).go(speed=DOWN_SPEED)
      yield self.nh.sleep(3)

      fprint('Going!')
      yield self.move.set_position(mid + 2 * norm).depth(DEPTH).go(speed=SPEED)
      yield self.nh.sleep(5)
      yield self.move.set_position(mid).depth(DEPTH).go(speed=SPEED)
      yield self.nh.sleep(5)
      yield self.move.set_position(mid - 2 * norm).depth(DEPTH).go(speed=SPEED)
      yield self.nh.sleep(5)
      for _ in range(6):
        yield self.move.yaw_right_deg(120).depth(DEPTH).go(speed=SPEED)
        yield self.nh.sleep(2)
#      yield self.move.forward(2).go(speed=SPEED)
