#!/usr/bin/env python
from __future__ import division

import os
import numpy as np
import yaml

import rospkg
from twisted.internet import defer
from txros import action, util, tf, NodeHandle
from pose_editor import PoseEditor2

from uf_common.msg import MoveToAction, PoseTwistStamped
from nav_msgs.msg import Odometry
from navigator_tools import rosmsg_to_numpy, odometry_to_numpy
import navigator_msgs.srv as navigator_srvs

class Navigator(object):
    def __init__(self, nh):
        self.nh = nh

        self._vision_proxies = {}
        self._load_vision_serivces()

    @util.cancellableInlineCallbacks
    def _init(self):
        self._moveto_action_client = yield action.ActionClient(self.nh, 'moveto', MoveToAction)
        self._odom_sub = yield self.nh.subscribe('odom', Odometry)
        self._ecef_odom_sub = yield self.nh.subscribe('absodom', Odometry)

        self._change_wrench = yield self.nh.get_service_client('/change_wrench', navigator_srvs.WrenchSelect)
        self.tf_listener = yield tf.TransformListener(self.nh)

        print "Waiting for odom..."
        yield self._odom_sub.get_next_message()  # We want to make sure odom is working before we continue

        defer.returnValue(self)

    @property
    def pose(self):
        last_odom_msg = self._odom_sub.get_last_message()
        return odometry_to_numpy(last_odom_msg)[0]

    @property
    def ecef_pose(self):
        last_odom_msg = self._ecef_odom_sub.get_last_message()
        return odometry_to_numpy(last_odom_msg)[0]

    @property
    def move(self):
        return PoseEditor2(self, 'enu')

    def change_wrench(self, source):
        return self._change_wrench(navigator_srvs.WrenchSelectRequest(source))

    def vision_request(self, request_name, **kwargs):
        assert request_name in self._vision_proxies.keys(), "Unknown request: {}".format(request_name)

        vision_proxy = self._vision_proxies[request_name]
        s_client = vision_proxy['client']
        s_args = dict(vision_proxy['args'].items() + kwargs.items())
        s_req = vision_proxy['request'](**s_args)

        # Returns deferred object, make sure to yield on this
        return s_client(s_req)

    def _load_vision_serivces(self, fname="vision_services.yaml"):
        rospack = rospkg.RosPack()
        config_file = os.path.join(rospack.get_path('navigator_missions'), 'navigator_singleton', fname)
        f = yaml.load(open(config_file, 'r'))
        for name in f:
            s_type = getattr(navigator_srvs, f[name]["type"])
            s_req = getattr(navigator_srvs, "{}Request".format(f[name]["type"]))
            s_client = self.nh.get_service_client(f[name]["topic"], s_type)
            self._vision_proxies[name] = {'client': s_client, 'request': s_req, 'args': f[name]['args']}

@util.cancellableInlineCallbacks
def main():
    nh = yield NodeHandle.from_argv("testing")
    n = yield _Navigator(nh)._init()
    print (yield n.vision_request('sonar'))


if __name__ == '__main__':
    from twisted.internet import defer, reactor
    import signal
    import traceback
    reactor.callWhenRunning(main)
    reactor.run()
