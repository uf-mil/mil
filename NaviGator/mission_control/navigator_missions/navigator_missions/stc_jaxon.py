#!/usr/bin/env python3
import asyncio

from axros import util
from navigator_msgs.msg import ScanTheCode

from .navigator import Navigator


class ScanTheCodeJaxon(Navigator):
    TIMEOUT_SECONDS = 30.0

    @classmethod
    async def init(cls):
        cls.stcsub = cls.nh.subscribe("/scan_the_code", ScanTheCode)
        cls.stcpub = cls.nh.advertise("/scan_the_code", ScanTheCode)
        await asyncio.gather(cls.stcsub.setup(), cls.stcpub.setup())

    @classmethod
    async def shutdown(cls):
        await asyncio.gather(cls.stcpub.shutdown(), cls.stcsub.shutdown())

    async def run(self, args):
        # Parse Arguments

        self.stc = await self.get_sorted_objects(name="stc_platform", n=1)
        self.stc = self.stc[1][0]

        await self.move.look_at(self.stc).go()

        await self.move.set_position(self.stc).backward(5).yaw_left(1.57).go()

        print("SELECTING")

        # Get scan the code stuff
        try:
            result = await util.wrap_time_notice(
                self.stcsub.get_next_message(), self.TIMEOUT_SECONDS, "test"
            )
            stc_result = result.color_pattern
        except util.TimeoutError:
            stc_result = "RGB"
            self.stcpub.publish(ScanTheCode(color_pattern=stc_result))

        self.net_stc_results = stc_result

        if stc_result[0] == "R":
            await self.move.look_at(self.stc).go()
            await self.move.left(10).forward(15).go()
        elif stc_result[0] == "G":
            await self.move.look_at(self.stc).go()
            await self.move.right(10).forward(15).go()
        elif stc_result[0] == "B":
            await self.move.left(5).go()
            await self.move.circle_point(
                self.stc, direction="cw", revolutions=1.25
            ).go()
            await self.move.forward(10).go()

        self.send_feedback("Done!")
