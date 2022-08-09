import genpy
import txros
from mil_misc_tools import text_effects
from ros_alarms import TxAlarmBroadcaster, TxAlarmListener
from twisted.internet import defer

from .bin_2022 import Bin2022
from .fire_torpedos_2022 import FireTorpedos2022
from .follow_path_2022 import FollowPath2022
from .make_the_grade_2022 import MakeGrade2022
from .pinger_2022 import Pinger2022

# Import missions here
from .start_gate_2022 import StartGate2022
from .sub_singleton import SubjuGator
from .surface import Surface

fprint = text_effects.FprintFactory(title="AUTO_MISSION").fprint
WAIT_SECONDS = 5.0


class Autonomous2022(SubjuGator):
    @txros.util.cancellableInlineCallbacks
    def run_mission(self, mission, timeout):
        # timeout in seconds
        m = mission.run(self).addErrback(lambda x: None)
        start_time = yield self.nh.get_time()
        while self.nh.get_time() - start_time < genpy.Duration(timeout):
            if len(m.callbacks) == 0:
                m.cancel()
                defer.returnValue(True)
            yield self.nh.sleep(0.5)
        fprint("MISSION TIMEOUT", msg_color="red")
        m.cancel()
        defer.returnValue(False)

    @txros.util.cancellableInlineCallbacks
    def do_mission(self):
        fprint("RUNNING MISSION", msg_color="blue")

        try:
            # Run start gate mission
            fprint("Running start gate mission", msg_color="green")
            yield self.run_mission(StartGate2022(), 400)

            # Run mission to follow orange marker
            fprint("Following orange marker", msg_color="green")
            yield self.run_mission(FollowPath2022(), 400)

            # Run mission to do phase 1 buoys
            fprint("Making the Grade", msg_color="green")
            yield self.run_mission(MakeGrade2022(), 400)

            # Run mission to follow orange marker
            fprint("Following orange marker", msg_color="green")
            yield self.run_mission(FollowPath2022(), 400)

            # Run mission to do bins mission
            fprint("Starting bins mission", msg_color="green")
            yield self.run_mission(Bin2022(), 400)

            # Run mission to get to next pinger location
            fprint("Going to pinger location", msg_color="green")
            yield self.run_mission(Pinger2022(), 400)

            # Run mission to Analyze where to fire torpedoes
            fprint("Starting mission to fire torpedoes", msg_color="green")
            yield self.run_mission(FireTorpedos2022(), 400)

            # Run mission to get to next pinger location
            fprint("Finding other pinger location", msg_color="green")
            yield self.run_mission(Pinger2022(), 400)

            # Rise from Octagon
            fprint("Rising from Octagon", msg_color="green")
            yield self.run_mission(Surface(), 400)

        except Exception as e:
            fprint("Error in Chain 1 missions!", msg_color="red")
            print(e)

        # Create a mission kill alarm and kill in the final area
        ab = yield TxAlarmBroadcaster.init(self.nh, "mission-kill")
        yield ab.raise_alarm()
        fprint("MISSION COMPLETE", msg_color="blue")

    @txros.util.cancellableInlineCallbacks
    def wait_before_start(self, nh):
        """Waits for the network loss alarm to trigger before"""
        if (yield nh.has_param("autonomous")):
            fprint(f"Waiting {WAIT_SECONDS} seconds before running missions...")

            yield nh.sleep(WAIT_SECONDS)
            fprint("Running Missions")
            yield self.do_mission()
        else:
            fprint("Please place sub in autonomous mode", msg_color="red")

    @txros.util.cancellableInlineCallbacks
    def run(self, args):

        yield self.wait_before_start(self.nh)
