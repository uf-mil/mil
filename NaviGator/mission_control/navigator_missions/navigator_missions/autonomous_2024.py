from __future__ import annotations

import asyncio
import datetime

from navigator_msgs.msg import ScanTheCode

from .go_to_poi import GoToPOI
from .navigator import NaviGatorMission
from .wildlife import Wildlife

# from .scan_the_code_2024 import ScanTheCode2024


class Autonomous2024(NaviGatorMission):

    async def run_mission(
        self,
        mission_cls: type[NaviGatorMission],
        name: str,
        *args,
        str_arg: str = "",
        timeout: int = 180,
        **kwargs,
    ):
        # self.send_feedback(f"beginning {name}...")
        try:
            await asyncio.wait_for(
                mission_cls().run(str_arg, *args, **kwargs),
                timeout,
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
        td_feedback_pub = self.nh.advertise("/scan_the_code", ScanTheCode)
        async with td_feedback_pub:
            self.start_time = datetime.datetime.now()
            self._last_checkpoint = self.start_time
            # self.send_feedback("proceeding to entrance gate...")
            # await self.go_to_poi("entrance_gate")
            # self.send_feedback("starting entrance gate task...")
            # await self.run_mission(
            #     EntranceGate2,
            #     "entrance gate",
            #     scan_code=True,
            #     return_to_start=False,
            #     circle=False,
            # )

            # Step 2: Scan the Code
            self.send_feedback("going to scan_the_code poi...")
            await self.go_to_poi("scan_the_code")
            # await self.run_mission(ScanTheCode2024, "scan the code")
            await asyncio.sleep(5)
            # fake publishing
            sequence = "RGB"
            self.send_feedback("reporting sequence...")
            td_feedback_pub.publish(ScanTheCode(color_pattern=sequence))
            await self.nh.set_param("color_sequence", sequence)

            # # Step 3: Wildlife Mission
            self.send_feedback("going to wildlife poi...")
            await self.go_to_poi("wildlife")
            self.send_feedback("running wildlife mission...")
            await self.run_mission(Wildlife, "wildlife", timeout=250)

            # Step 4: Navigation Mission
            # await self.run_mission(Navigation, "navigation")

            # Step 5: Dock Mission
            # await self.run_mission(Docking, "docking")

            # Step 6: Exit through the same gate
            self.send_feedback("returning through same gate...")
            await self.go_to_poi("exit_gate")
            # await self.run_mission(
            #     EntranceGate2,
            #     "same gate",
            #     scan_code=True,
            #     return_to_start=False,
            #     circle=False,
            # )
            # Go could be safer than attempting to find the best gate, just remember
            # to send heartbeat
            await self.move.forward(12).go()
