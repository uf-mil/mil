#!/usr/bin/env python3
import numpy as np
from geometry_msgs.msg import Point
from mil_misc_tools import ThrowingArgumentParser
from mil_msgs.msg import ObjectsInImage
from mil_msgs.srv import CameraToLidarTransform, CameraToLidarTransformRequest
from mil_tools import rosmsg_to_numpy
from twisted.internet import defer
from txros import util

from .navigator import Navigator


class ShootBalls(Navigator):
    @util.cancellableInlineCallbacks
    def run(self, args):
        for i in range(0, 4):
            yield self.reload_launcher()
            yield self.nh.sleep(2)
            yield self.fire_launcher()
            yield self.nh.sleep(2)
        yield self.set_vision_off()
