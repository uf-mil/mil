#!/usr/bin/env python
import txros
import rospy
import numpy as np
from vrx import Vrx
from vrx_gazebo.srv import ColorSequenceRequest, ColorSequence
from std_msgs.msg import String
from nav_msgs.msg import Odometry
from mil_tools import numpy_to_pointcloud2 as np2pc2, rosmsg_to_numpy


class ScanAndDock(Vrx):

    def __init__(self, *args, **kwargs):
        super(ScanAndDock, self).__init__(*args, **kwargs)

    @txros.util.cancellableInlineCallbacks
    def run(self, args):
        yield self.nh.sleep(5)

        sequence = yield self.run_submission('ScanTheCode')
        color_to_shape = {'red': 'circle', 'green' : 'triangle', 'blue' : 'cross', 'yellow' : 'rectangle'}

        try:
            yield self.run_submission('Dock', parameters='%s %s'%(sequence[0], color_to_shape[sequence[2]]))
        except Exception as e:
            print(e)
        yield self.send_feedback('Done!')
