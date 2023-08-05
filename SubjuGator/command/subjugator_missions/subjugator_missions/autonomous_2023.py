import genpy
from mil_misc_tools import text_effects
from ros_alarms import TxAlarmBroadcaster

# Import missions here
from .coin_flip import CoinFlip
from .start_gate_style import StartGateStyle
from .octagon import Octagon
from .surface import Surface
from .sub_singleton import SubjuGatorMission

fprint = text_effects.FprintFactory(title="AUTO_MISSION_2023").fprint
WAIT_SECONDS = 5.0


class Autonomous2023(SubjuGatorMission):
    async def run_mission(self, mission, timeout):
        # timeout in seconds
        m = mission.run(self)
        start_time = self.nh.get_time()
        while self.nh.get_time() - start_time < genpy.Duration(timeout):
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
            # Do coin flip turn
            fprint("Running coin flip mission", msg_color="green")
            await self.run_mission(CoinFlip(), 50)

            # Run start gate style mission
            fprint("Running start gate style mission", msg_color="green")
            await self.run_mission(StartGateStyle(), 200)

            # Run octagon mission
            fprint("Running octagon mission", msg_color="green")
            await self.run_mission(Octagon(), 400)

            # Rise from Octagon
            fprint("Rising from Octagon", msg_color="green")
            await self.run_mission(Surface(), 120)

        except Exception as e:
            fprint("Error in Chain 1 missions!", msg_color="red")
            print(e)

        # Create a mission kill alarm and kill in the final area
        ab = await TxAlarmBroadcaster.init(self.nh, "mission-kill")
        await ab.raise_alarm()
        fprint("MISSION COMPLETE", msg_color="blue")

    async def wait_before_start(self, nh):
        """Waits for the network loss alarm to trigger before"""
        if await nh.has_param("autonomous"):
            fprint(f"Waiting {WAIT_SECONDS} seconds before running missions...")

            await nh.sleep(WAIT_SECONDS)
            fprint("Running Missions")
            await self.do_mission()
        else:
            fprint("Please place sub in autonomous mode", msg_color="red")

    async def run(self, args):
        await self.wait_before_start(self.nh)
