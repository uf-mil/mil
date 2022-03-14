#!/usr/bin/env python
import txros
import rospy
import numpy as np
from vrx import Vrx
from vrx_gazebo.srv import ColorSequenceRequest, ColorSequence
from std_msgs.msg import String
from nav_msgs.msg import Odometry
from mil_tools import numpy_to_pointcloud2 as np2pc2, rosmsg_to_numpy


class Gymkhana(Vrx):

    def __init__(self, *args, **kwargs):
        super(Gymkhana, self).__init__(*args, **kwargs)

    @txros.util.cancellableInlineCallbacks
    def run(self, args):
        yield self.nh.sleep(5)

        yield self.run_submission('VrxNavigation')
        yield self.run_submission('VrxBeacon')
        yield self.run_submission('VrxBeacon')
        yield self.send_feedback('Done!')
