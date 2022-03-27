#!/usr/bin/env python
import txros
import rospy
import numpy as np
from vrx import Vrx
from vrx_gazebo.srv import ColorSequenceRequest, ColorSequence
from navigator_msgs.srv import AverageScaleRequest
from std_msgs.msg import String
from nav_msgs.msg import Odometry
from mil_tools import numpy_to_pointcloud2 as np2pc2, rosmsg_to_numpy


class Gymkhana(Vrx):

    def __init__(self, *args, **kwargs):
        super(Gymkhana, self).__init__(*args, **kwargs)

    @txros.util.cancellableInlineCallbacks
    def run(self, args):
        yield self.nh.sleep(5)


        yield self.move.yaw_left(45,"deg").go()

        for j in range(10):

            scales = yield self.pcodar_scales(AverageScaleRequest())

            print("yoooo",len(scales.objects))

            f = open("datapoints.csv", "w")
            for i,scale in enumerate(scales.objects):

                scale_x_avg = scale.scale_x
                scale_y_avg = scale.scale_y
                scale_z_avg = scale.scale_z

                if scale_z_avg > 0.35:
                    f.write(str(i) + "," + str(scale_z_avg) + "," + "cone buoy" + "\n")
                else:
                    f.write(str(i) + "," + str(scale_z_avg) + "," + "round buoy" + "\n")

                #f.write(str(dist) + "," + str(scale_x) + "," + str(scale_y) + "," + str(scale_z) + "\n")

            f.close()
            yield self.move.forward(5, 'm').go()

        #yield self.run_submission('VrxNavigation')
        #yield self.run_submission('VrxBeacon')
        #yield self.run_submission('VrxBeacon')
        yield self.send_feedback('Done!')
