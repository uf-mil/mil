from __future__ import annotations

import asyncio
import datetime

import axros
from navigator_msgs.msg import ScanTheCode
from navigator_msgs.srv import MessageDetectDock, MessageDetectDockRequest
from std_srvs.srv import Empty, EmptyRequest, SetBool, SetBoolRequest

from .go_to_poi import GoToPOI
from .navigator import NaviGatorMission

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
        stc_pub = self.nh.advertise("/stc_display", ScanTheCode)
        async with td_feedback_pub, stc_pub:
            stc_pub.publish(ScanTheCode(color_pattern="XXX"))
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
            # self.send_feedback("going to scan_the_code poi...")
            # await self.go_to_poi("scan_the_code")
            # # await self.run_mission(ScanTheCode2024, "scan the code")
            # await asyncio.sleep(5)
            # # fake publishing
            # sequence = "RGB"
            # self.send_feedback("reporting sequence...")
            # td_feedback_pub.publish(ScanTheCode(color_pattern=sequence))
            # stc_pub.publish(ScanTheCode(color_pattern=sequence))
            # await self.nh.set_param("color_sequence", sequence)

            # # # Step 3: Wildlife Mission
            # self.send_feedback("going to wildlife poi...")
            # await self.go_to_poi("wildlife")
            # self.send_feedback("running wildlife mission...")
            # await self.run_mission(Wildlife, "wildlife", timeout=250)

            # Step 4: Navigation Mission
            # self.send_feedback("going to navigation poi...")
            # await self.go_to_poi("navigation")
            # await self.run_mission(NavigationGatefinder, "navigation", timeout=240)

            # Step 5: Dock Mission
            self.send_feedback("going to docking poi...")
            await self.go_to_poi("docking")
            # await self.run_mission(Docking, "docking")
            await self.move.forward(10).go(speed_factor=0.5)
            spin_srv = self.nh.get_service_client("/ball_launcher/spin", SetBool)
            drop_srv = self.nh.get_service_client("/ball_launcher/drop_ball", Empty)
            dock_heartbeat = self.nh.get_service_client(
                "/detect_dock_message",
                MessageDetectDock,
            )
            try:
                await axros.wrap_timeout(dock_heartbeat.wait_for_service(), duration=5)
                await dock_heartbeat(
                    MessageDetectDockRequest(
                        color="R",
                        ams_status=1,
                        status_of_delivery="S",
                    ),
                )
            except TimeoutError:
                self.send_feedback("dock heartbeat service not available")
            try:
                await axros.wrap_timeout(spin_srv.wait_for_service(), duration=5)
                await axros.wrap_timeout(drop_srv.wait_for_service(), duration=5)
                await spin_srv(SetBoolRequest(True))
                await asyncio.sleep(4)
                for _ in range(4):
                    await drop_srv(EmptyRequest())
                    await asyncio.sleep(1.5)
                await spin_srv(SetBoolRequest(False))
            except TimeoutError:
                pass
            try:
                await axros.wrap_timeout(dock_heartbeat.wait_for_service(), duration=5)
                await dock_heartbeat(
                    MessageDetectDockRequest(
                        color="R",
                        ams_status=1,
                        status_of_delivery="S",
                    ),
                )
            except TimeoutError:
                self.send_feedback("dock heartbeat service not available")
            await self.move.backward(10).go(speed_factor=0.5)

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
