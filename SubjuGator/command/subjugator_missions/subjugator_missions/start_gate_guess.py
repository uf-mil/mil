import numpy as np
import rospy
from mil_misc_tools import text_effects
from std_srvs.srv import Trigger

from .sub_singleton import SubjuGatorMission

fprint = text_effects.FprintFactory(title="PINGER", msg_color="cyan").fprint

SPEED = 0.6
DOWN_SPEED = 0.1

DOWN = 1.5


class StartGateGuess(SubjuGatorMission):
    async def run(self, args):
        fprint("Getting Guess Locations")

        sub_start_position, sub_start_orientation = await self.tx_pose()

        save_pois = rospy.ServiceProxy("/poi_server/save_to_param", Trigger)
        _ = save_pois()
        gate_1 = np.array(rospy.get_param("/poi_server/initial_pois/start_gate1"))
        np.array(rospy.get_param("/poi_server/initial_pois/start_gate2"))
        # mid = (gate_1 + gate_2) / 2
        # mid = gate_1
        fprint(f"Found mid {gate_1}")

        fprint("Looking at gate")
        #      await self.move.down(DOWN).set_orientation(sub_start_orientation).go(
        #########speed=DOWN_SPEED)
        #      await self.move.look_at_without_pitching(mid).go(speed=DOWN_SPEED)

        fprint("Going!")
        await self.go(self.move().set_position(gate_1).depth(DOWN), speed=SPEED)
        await self.go(self.move().forward(1), speed=SPEED)
