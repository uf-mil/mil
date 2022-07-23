#!/usr/bin/env python3
import numpy as np
import rospy
import txros
from mil_tools import numpy_to_pointcloud2 as np2pc2
from mil_tools import rosmsg_to_numpy
from nav_msgs.msg import Odometry
from std_msgs.msg import String
from vrx import Vrx
from vrx_gazebo.srv import ColorSequence, ColorSequenceRequest


class ScanAndDock(Vrx):
    def __init__(self, *args, **kwargs):
        super().__init__(*args, **kwargs)

    @txros.util.cancellableInlineCallbacks
    def run(self, args):
        yield self.nh.sleep(10)
        yield self.wait_for_task_such_that(
            lambda task: task.state in ["ready", "running"]
        )

        sequence = yield self.run_submission("ScanTheCode")
        color_to_shape = {
            "red": "circle",
            "green": "triangle",
            "blue": "cross",
            "yellow": "rectangle",
        }

        try:
            yield self.run_submission(
                "Dock",
                parameters=f"{sequence[0]} {color_to_shape[sequence[2]]}",
            )
        except Exception as e:
            print(e)
        yield self.send_feedback("Done!")
