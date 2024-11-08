from __future__ import annotations

import asyncio

import rospy

from .docking import Docking
from .entrance_gate2 import EntranceGate2
from .go_to_poi import GoToPOI
from .navigation import Navigation
from .navigator import NaviGatorMission
from .scan_the_code import ScanTheCodeMission
from .wildlife import Wildlife


class Autonomous2024(NaviGatorMission):

    # timeout (in secs)
    TIMEOUT = 180

    async def run_mission(
        self,
        mission_cls: type[NaviGatorMission],
        name: str,
        *args,
        str_arg: str = "",
        **kwargs,
    ):
        rospy.loginfo(f"[autonomous] beginning {name}...")
        try:
            await asyncio.wait_for(
                mission_cls().run(str_arg, *args, **kwargs), self.TIMEOUT,
            )
        except asyncio.TimeoutError:
            rospy.logwarn(f"[autonomous] ran out of time on {name}!")

    async def run(self, args: str):
        # Step 1: Entrance and exit gates
        await self.send_feedback("[AUTONOMOUS] GOING TO ENTRANCE GATE")
        await self.run_mission(GoToPOI, "entrance_gate", str_arg="entrance_gate")
        await self.send_feedback("[AUTONOMOUS] STARTING AUTONOMOUS MISSION")
        await self.run_mission(EntranceGate2, "entrance gate", scan_code=True)

        # Step 1.5: Launch the drone
        # FILL IN HERE
        await self.send_feedback(
            "[AUTONOMOUS] LAUNCHING THE DRONE, T-45 SEC TO CONTINUE",
        )
        await asyncio.sleep(10)
        await self.send_feedback(
            "[AUTONOMOUS] LAUNCHING THE DRONE, T-35 SEC TO CONTINUE",
        )
        await asyncio.sleep(10)
        await self.send_feedback(
            "[AUTONOMOUS] LAUNCHING THE DRONE, T-25 SEC TO CONTINUE",
        )
        await asyncio.sleep(10)
        await self.send_feedback(
            "[AUTONOMOUS] LAUNCHING THE DRONE, T-15 SEC TO CONTINUE",
        )
        await asyncio.sleep(10)
        await self.send_feedback(
            "[AUTONOMOUS] LAUNCHING THE DRONE, T-5 SEC TO CONTINUE",
        )
        await asyncio.sleep(5)

        # Step 2: Scan the Code
        await self.send_feedback("[AUTONOMOUS] GOING TO SCAN THE CODE")
        await self.run_mission(GoToPOI, "scan_the_code poi", str_arg="scan_the_code")
        await self.run_mission(ScanTheCodeMission, "scan the code")

        # Step 3: Wildlife Mission
        await self.send_feedback("[AUTONOMOUS] GOING TO WILDLIFE")
        await self.run_mission(GoToPOI, "wildlife poi", str_arg="wildlife")
        await self.run_mission(Wildlife, "wildlife")

        # Step 4: Navigation Mission
        await self.send_feedback("[AUTONOMOUS] GOING TO NAVIGATION")
        await self.run_mission(GoToPOI, "navigation poi", str_arg="navigation")
        await self.run_mission(Navigation, "navigation")

        # Step 4.5: Receive the drone
        # FILL IN HERE

        # Step 5: Dock Mission
        await self.send_feedback("[AUTONOMOUS] GOING TO DOCKING")
        await self.run_mission(GoToPOI, "docking poi", str_arg="docking")
        await self.run_mission(Docking, "docking")

        # Step 6: Leave the course
        await self.send_feedback("[AUTONOMOUS] LEAVING COURSE")
        await self.run_mission(GoToPOI, "exit_gate poi", str_arg="exit_gate")
