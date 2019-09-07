from txros import util
import rospy
from std_srvs.srv import Trigger
import numpy as np
import mil_ros_tools
from mil_misc_tools import text_effects
from sub8_msgs.srv import GuessRequest, GuessRequestRequest
from .sub_singleton import SubjuGator

fprint = text_effects.FprintFactory(title="PINGER", msg_color="cyan").fprint

SPEED = 0.6
DOWN_SPEED = 0.1

DOWN = 1.5


class StartGateGuess(SubjuGator):
    @util.cancellableInlineCallbacks
    def run(self, args):

      fprint('Getting Guess Locations')

      sub_start_position, sub_start_orientation = yield self.tx_pose()

      save_pois = rospy.ServiceProxy(
                '/poi_server/save_to_param', Trigger)
      _ = save_pois();
      gate_1 = np.array(rospy.get_param('/poi_server/initial_pois/start_gate1'))
      gate_2 = np.array(rospy.get_param('/poi_server/initial_pois/start_gate2'))
      #mid = (gate_1 + gate_2) / 2
      #mid = gate_1
      fprint('Found mid {}'.format(gate_1))

      fprint('Looking at gate')
#      yield self.move.down(DOWN).set_orientation(sub_start_orientation).go(
          #########speed=DOWN_SPEED)
#      yield self.move.look_at_without_pitching(mid).go(speed=DOWN_SPEED)

      fprint('Going!')
      yield self.move.set_position(gate_1).depth(DOWN).go(speed=SPEED)
      yield self.move.forward(1).go(speed=SPEED)
