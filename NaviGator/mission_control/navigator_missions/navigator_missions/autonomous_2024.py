from __future__ import annotations

import asyncio

import rospy

from .docking import Docking
from .entrance_gate2 import EntranceGate2
from .navigation import Navigation
from .navigator import NaviGatorMission
from .scan_the_code import ScanTheCodeMission
from .wildlife import Wildlife


class Autonomous2024(NaviGatorMission):

    # timeout (in secs)
    TIMEOUT = 180

    async def run_mission(self, mission_cls: type[NaviGatorMission], name: str):
        rospy.loginfo(f"[autonomous] beginning {name}...")
        try:
            await asyncio.wait_for(mission_cls().run(""), self.TIMEOUT)
        except asyncio.TimeoutError:
            rospy.logwarn(f"[autonomous] ran out of time on {name}!")

    async def run(self, args: str):
        # Step 1: Entrance and exit gates
        await self.run_mission(EntranceGate2, "entrance gate")

        # Step 2: Scan the Code
        await self.run_mission(ScanTheCodeMission, "scan the code")

        # Step 3: Wildlife Mission
        await self.run_mission(Wildlife, "wildlife")

        # Step 4: Navigation Mission
        await self.run_mission(Navigation, "navigation")

        # Step 5: Dock Mission
        await self.run_mission(Docking, "docking")

        # Step 6: UAV Mission
        pass
