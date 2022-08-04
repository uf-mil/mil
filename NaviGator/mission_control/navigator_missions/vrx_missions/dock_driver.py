#!/usr/bin/env python3
import numpy as np
import txros
from std_msgs.msg import String

from .vrx import Vrx


class DockDriver(Vrx):
    def __init__(self, *args, **kwargs):
        super().__init__(*args, **kwargs)

    @txros.util.cancellableInlineCallbacks
    def run(self, args):
        yield self.nh.sleep(5)
        self.scan_dock_placard_symbol = self.nh.subscribe(
            "/vrx/scan_dock/placard_symbol", String
        )
        dock_msg = yield self.scan_dock_placard_symbol.get_next_message()
        dock_msg = str.split(dock_msg.data, "_")
        print(dock_msg)
        shape_to_shape = {
            "circle": "circle",
            "triangle": "triangle",
            "cross": "cruciform",
        }

        try:
            yield self.run_submission(
                "Dock",
                parameters=f"{dock_msg[0]} {shape_to_shape[dock_msg[1]]}",
            )
        except Exception as e:
            print(e)
        yield self.send_feedback("Done!")
