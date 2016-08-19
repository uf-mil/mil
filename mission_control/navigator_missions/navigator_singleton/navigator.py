#!/usr/bin/env python
from __future__ import division
import os
import numpy as np
from twisted.internet import defer
from txros import action, util, tf, serviceclient, NodeHandle
from uf_common.msg import MoveToAction, PoseTwistStamped
from nav_msgs.msg import Odometry
from navigator_tools import rosmsg_to_numpy, odometry_to_numpy
from navigator_msgs.srv import *
# from navigator import pose_editor
from pose_editor import PoseEditor2
import rospkg
import yaml


class Navigator(object):
    def __init__(self, nh):
        self.nh = nh

        self._vision_proxies = {}
        self._load_vision_serivces()

    @util.cancellableInlineCallbacks
    def _init(self):
        self._moveto_action_client = yield action.ActionClient(self.nh, 'moveto', MoveToAction)
        self._odom_sub = yield self.nh.subscribe('odom', Odometry)

        #self._enu_odom_sub = yield self.nh.subscribe('world_odom', Odometry)
        self.tf_listener = yield tf.TransformListener(self.nh)

        yield self.nh.sleep(.3)  # Build tf and odom buffers
        defer.returnValue(self)

    @property
    def pose(self):
        # TODO: Fails to get last message when
        last_odom_msg = self._odom_sub.get_last_message()
        return odometry_to_numpy(last_odom_msg)[0]

    @property
    def move_rel(self):
        return PoseEditor2(self._moveto_action_client, 'base_link', [0, 0, 0], [0, 0, 0 ,1], self.tf_listener)

    @property
    def move(self):
        return PoseEditor2(self._moveto_action_client, 'enu', *self.pose)

    def move_to(self, frame_id, position, orientation=[0, 0, 0, 1]):
        # TOOD
        # ps = PoseStamped()
        # self.tf_listenerf
        raise NotImplementedError
        #return PoseEditor2(self.pose)

    @util.cancellableInlineCallbacks
    def vision_request(self, request_name, **kwargs):
        assert request_name in self._vision_proxies.keys(), "Unknown request: {}".format(request_name)

        vision_proxy = self._vision_proxies[request_name]
        s_client = vision_proxy['client']
        s_args = dict(vision_proxy['args'].items() + kwargs.items())
        s_req = vision_proxy['request'](**s_args)

        resp = yield s_client(s_req)
        defer.returnValue(resp)

    def _load_vision_serivces(self, fname="vision_services.yaml"):
        rospack = rospkg.RosPack()
        config_file = os.path.join(rospack.get_path('navigator_missions'), 'navigator_singleton', fname)
        f = yaml.load(open(config_file, 'r'))
        for name in f:
            # Maybe get getattr here
            s_type = eval(f[name]["type"])
            s_req = eval("{}Request".format(f[name]["type"]))
            s_client = self.nh.get_service_client(f[name]["topic"], s_type)
            self._vision_proxies[name] = {'client': s_client, 'request': s_req, 'args': f[name]['args']}

    # @util.cancellableInlineCallbacks
    # def last_pose(self):
    #     last_pose = yield self._odom_sub.get_next_message()
    #     defer.returnValue(last_pose)

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
