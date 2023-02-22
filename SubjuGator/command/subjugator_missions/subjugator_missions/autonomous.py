import asyncio

import axros
import genpy
from mil_misc_tools import text_effects
from ros_alarms import TxAlarmBroadcaster, TxAlarmListener

from .arm_torpedos import FireTorpedos
from .ball_drop import BallDrop
from .pinger import Pinger

# Import missions here
from .start_gate import StartGate
from .sub_singleton import SubjuGatorMission
from .surface import Surface
from .vampire_slayer import VampireSlayer

fprint = text_effects.FprintFactory(title="AUTO_MISSION").fprint
WAIT_SECONDS = 5.0


class Autonomous(SubjuGatorMission):
    async def run_mission(self, mission, timeout):
        # timeout in seconds
        m = mission.run(self)
        start_time = self.nh.get_time()
        while self.nh.get_time() - start_time < genpy.Duration(timeout):
            # oof what a hack
            if len(m.callbacks) == 0:
                m.cancel()
                return True
            await self.nh.sleep(0.5)
        fprint("MISSION TIMEOUT", msg_color="red")
        m.cancel()
        return False

    async def do_mission(self):
        fprint("RUNNING MISSION", msg_color="blue")

        try:
            # Run start gate mission
            await self.run_mission(StartGate(), 400)

            # Go to pinger and do corresponding mission
            completed = await self.run_mission(Pinger(), 400)
            if not completed:  # if we timeout
                pass
            else:
                if await self.nh.has_param("pinger_where"):
                    if (await self.nh.get_param("pinger_where")) == 0:
                        fprint("Surface Mission")
                        await self.run_mission(Surface(), 30)
                    elif (await self.nh.get_param("pinger_where")) == 1:
                        fprint("Shooting Mission")
                        await self.run_mission(FireTorpedos(), 400)

            # Go to the other pinger mission and do respective mission
            completed = await self.run_mission(Pinger(), 400)
            if not completed:  # if we timeout
                pass
            else:
                if await self.nh.has_param("pinger_where"):
                    if (await self.nh.get_param("pinger_where")) == 0:
                        fprint("Surface Mission")
                        await self.run_mission(Surface(), 30)

                    elif (await self.nh.get_param("pinger_where")) == 1:
                        fprint("Shooting Mission")
                        await self.run_mission(FireTorpedos(), 400)

            fprint("Vampire Slayer")
            await self.run_mission(VampireSlayer(), 400)
            fprint("Garlic drop?")
            await self.rub_mission(BallDrop(), 400)

        except Exception as e:
            fprint("Error in Chain 1 missions!", msg_color="red")
            print(e)

        # Create a mission kill alarm and kill in the final area
        ab = await TxAlarmBroadcaster.init(self.nh, "mission-kill")
        await ab.raise_alarm()
        fprint("MISSION COMPLETE", msg_color="green")

    async def _check_for_run(self, nh: axros.NodeHandle, _):
        """Waits for the network loss alarm to trigger before"""
        if (await nh.has_param("autonomous")) and (await nh.get_param("autonomous")):
            fprint(f"Waiting {WAIT_SECONDS} seconds before running missions...")
            await nh.sleep(WAIT_SECONDS)
            fprint("Running Missions")
            await self.do_mission()
        else:
            fprint("Network loss deteceted but NOT starting mission.", msg_color="red")

    async def _auto_param_watchdog(self, nh: axros.NodeHandle):
        """
        Watch the `autonomous` param and notify the user when events happen.
        """
        ready = False
        while True:
            await nh.sleep(0.1)

            if not (await nh.has_param("autonomous")):
                if ready:
                    ready = False
                    fprint("Autonomous mission disarmed.")
                continue

            if (await nh.get_param("autonomous")) and not ready:
                ready = True
                fprint(
                    "Autonomous mission armed. Disconnect now to run.",
                    msg_color="yellow",
                )

            elif not (await nh.get_param("autonomous")) and ready:
                ready = False
                fprint("Autonomous mission disarmed.")

    async def run(self, args):
        al = await TxAlarmListener.init(self.nh, "network-loss")
        await self._auto_param_watchdog(self.nh)

        call_with_sub = lambda *args: self._check_for_run(*args)
        al.add_callback(call_with_sub, call_when_cleared=False)

        fprint("Waiting for network-loss...", msg_color="blue")
        await asyncio.Future()
