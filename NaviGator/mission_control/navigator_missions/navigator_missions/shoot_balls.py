#!/usr/bin/env python
from txros import util
from navigator_missions.navigator import Navigator
import numpy as np
from mil_tools import rosmsg_to_numpy
from twisted.internet import defer
from mil_misc_tools import ThrowingArgumentParser
from mil_msgs.srv import CameraToLidarTransform, CameraToLidarTransformRequest
from mil_msgs.msg import ObjectsInImage
from geometry_msgs.msg import Point


class ShootBalls(Navigator):
    @util.cancellableInlineCallbacks
    def run(self, args):
        for i in range(0, 4):
            yield self.reload_launcher()
            yield self.nh.sleep(2)
            yield self.fire_launcher()
            yield self.nh.sleep(2)
        yield self.set_vision_off()