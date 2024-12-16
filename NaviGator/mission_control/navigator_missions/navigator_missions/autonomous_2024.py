from __future__ import annotations

import asyncio
import datetime

from .entrance_gate2 import EntranceGate2
from .go_to_poi import GoToPOI
from .navigator import NaviGatorMission
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
        # self.send_feedback(f"beginning {name}...")
        try:
            await asyncio.wait_for(
                mission_cls().run(str_arg, *args, **kwargs),
                self.TIMEOUT,
            )
        except asyncio.TimeoutError:
            self.send_feedback(f"!!! ran out of time on {name}!")

    async def go_to_poi(self, name: str):
        await self.run_mission(GoToPOI, f"go to {name} poi", str_arg=name)

    def time_elapsed(self) -> str:
        last_min, last_sec = divmod(
            (datetime.datetime.now() - self._last_checkpoint).total_seconds(),
            60,
        )
        min, sec = divmod(
            (datetime.datetime.now() - self.start_time).total_seconds(),
            60,
        )
        self._last_checkpoint = datetime.datetime.now()
        return f"{int(min)}m {int(sec)}s (+{int(last_min)}m {int(last_sec)}s)"

    def send_feedback(self, msg: str):
        super().send_feedback(f"[autonomous, {self.time_elapsed()}] {msg}")

    async def run(self, args: str):
        # Step 1: Entrance gate
        self.start_time = datetime.datetime.now()
        self._last_checkpoint = self.start_time
        self.send_feedback("proceeding to entrance gate...")
        await self.go_to_poi("entrance_gate")
        self.send_feedback("starting entrance gate task...")
        await self.run_mission(
            EntranceGate2,
            "entrance gate",
            scan_code=True,
            return_to_start=False,
            circle=False,
        )

        # Step 2: Scan the Code
        # await self.run_mission(ScanTheCodeMission, "scan the code")

        # Step 3: Wildlife Mission
        self.send_feedback("going to wildlife poi...")
        await self.go_to_poi("wildlife")
        self.send_feedback("running wildlife mission...")
        await self.run_mission(Wildlife, "wildlife")

        # Step 4: Navigation Mission
        # await self.run_mission(Navigation, "navigation")

        # Step 5: Dock Mission
        # await self.run_mission(Docking, "docking")

        # Step 6: Exit through the same gate
