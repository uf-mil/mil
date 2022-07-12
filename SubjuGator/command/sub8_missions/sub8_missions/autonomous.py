import genpy
import txros
from mil_misc_tools import text_effects
from ros_alarms import TxAlarmBroadcaster, TxAlarmListener
from twisted.internet import defer

from .arm_torpedos import FireTorpedos
from .ball_drop import BallDrop
from .pinger import Pinger

# Import missions here
from .start_gate import StartGate
from .sub_singleton import SubjuGator
from .surface import Surface
from .vampire_slayer import VampireSlayer

fprint = text_effects.FprintFactory(title="AUTO_MISSION").fprint
WAIT_SECONDS = 5.0


class Autonomous(SubjuGator):
    @txros.util.cancellableInlineCallbacks
    def run_mission(self, mission, timeout):
        # timeout in seconds
        m = mission.run(self).addErrback(lambda x: None)
        start_time = yield self.nh.get_time()
        while self.nh.get_time() - start_time < genpy.Duration(timeout):
            # oof what a hack
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
            yield self.run_mission(StartGate(), 400)

            # Go to pinger and do corresponding mission
            completed = yield self.run_mission(Pinger(), 400)
            if not completed:  # if we timeout
                pass
            else:
                if (yield self.nh.has_param("pinger_where")):
                    if (yield self.nh.get_param("pinger_where")) == 0:
                        fprint("Surface Mission")
                        yield self.run_mission(Surface(), 30)
                    elif (yield self.nh.get_param("pinger_where")) == 1:
                        fprint("Shooting Mission")
                        yield self.run_mission(FireTorpedos(), 400)

            # Go to the other pinger mission and do respective mission
            completed = yield self.run_mission(Pinger(), 400)
            if not completed:  # if we timeout
                pass
            else:
                if (yield self.nh.has_param("pinger_where")):
                    if (yield self.nh.get_param("pinger_where")) == 0:
                        fprint("Surface Mission")
                        yield self.run_mission(Surface(), 30)

                    elif (yield self.nh.get_param("pinger_where")) == 1:
                        fprint("Shooting Mission")
                        yield self.run_mission(FireTorpedos(), 400)

            fprint("Vampire Slayer")
            yield self.run_mission(VampireSlayer(), 400)
            fprint("Garlic drop?")
            yield self.rub_mission(BallDrop(), 400)

        except Exception as e:
            fprint("Error in Chain 1 missions!", msg_color="red")
            print(e)

        # Create a mission kill alarm and kill in the final area
        ab = yield TxAlarmBroadcaster.init(self.nh, "mission-kill")
        yield ab.raise_alarm()
        fprint("MISSION COMPLETE", msg_color="green")

    @txros.util.cancellableInlineCallbacks
    def _check_for_run(self, nh: txros.NodeHandle, _):
        """Waits for the network loss alarm to trigger before"""
        if (yield nh.has_param("autonomous")) and (yield nh.get_param("autonomous")):
            fprint(f"Waiting {WAIT_SECONDS} seconds before running missions...")
            yield nh.sleep(WAIT_SECONDS)
            fprint("Running Missions")
            yield self.do_mission()
        else:
            fprint("Network loss deteceted but NOT starting mission.", msg_color="red")

    @txros.util.cancellableInlineCallbacks
    def _auto_param_watchdog(self, nh: txros.NodeHandle):
        """
        Watch the `autonomous` param and notify the user when events happen.
        """
        ready = False
        while True:
            yield nh.sleep(0.1)

            if not (yield nh.has_param("autonomous")):
                if ready:
                    ready = False
                    fprint("Autonomous mission disarmed.")
                continue

            if (yield nh.get_param("autonomous")) and not ready:
                ready = True
                fprint(
                    "Autonomous mission armed. Disconnect now to run.",
                    msg_color="yellow",
                )

            elif not (yield nh.get_param("autonomous")) and ready:
                ready = False
                fprint("Autonomous mission disarmed.")

    @txros.util.cancellableInlineCallbacks
    def run(self, args):
        al = yield TxAlarmListener.init(self.nh, "network-loss")
        self._auto_param_watchdog(self.nh)

        call_with_sub = lambda *args: self._check_for_run(*args)
        al.add_callback(call_with_sub, call_when_cleared=False)

        fprint("Waiting for network-loss...", msg_color="blue")
        yield defer.Deferred()
