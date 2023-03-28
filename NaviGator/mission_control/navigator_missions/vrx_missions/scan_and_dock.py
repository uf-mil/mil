#!/usr/bin/env python3
from .vrx import Vrx


class ScanAndDock(Vrx):
    def __init__(self, *args, **kwargs):
        super().__init__(*args, **kwargs)

    async def run(self, args):
        await self.nh.sleep(10)
        await self.wait_for_task_such_that(
            lambda task: task.state in ["ready", "running"],
        )

        sequence = await self.run_submission("ScanTheCode")
        color_to_shape = {
            "red": "circle",
            "green": "triangle",
            "blue": "cross",
            "yellow": "rectangle",
        }

        try:
            await self.run_submission(
                "Dock",
                parameters=f"{sequence[0]} {color_to_shape[sequence[2]]}",
            )
        except Exception as e:
            print(e)
        await self.send_feedback("Done!")
