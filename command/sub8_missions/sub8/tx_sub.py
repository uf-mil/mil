from __future__ import division

import numpy as np
from twisted.internet import defer
from txros import action, util, tf
from uf_common.msg import MoveToAction, PoseTwistStamped, Float64Stamped
from uf_common import orientation_helpers
from nav_msgs.msg import Odometry


class _PoseProxy(object):
    def __init__(self, sub, pose):
        self._sub = sub
        self._pose = pose

    def __getattr__(self, name):
        def sub_attr_proxy(*args, **kwargs):
            return _PoseProxy(self._sub, getattr(self._pose, name)(*args, **kwargs))
        return sub_attr_proxy

    def go(self, *args, **kwargs):
        goal = self._sub._moveto_action_client.send_goal(self._pose.as_MoveToGoal(*args, **kwargs))
        return goal.get_result()

    def go_trajectory(self, *args, **kwargs):
        traj = self._sub._trajectory_pub.publish(
            self._pose.as_PoseTwistStamped(*args, **kwargs)
        )
        return traj


class _Sub(object):
    def __init__(self, node_handle):
        self._node_handle = node_handle

    @util.cancellableInlineCallbacks
    def _init(self):
        self._moveto_action_client = yield action.ActionClient(self._node_handle, 'moveto', MoveToAction)
        self._odom_sub = yield self._node_handle.subscribe('odom', Odometry)
        self._trajectory_sub = yield self._node_handle.subscribe('trajectory', PoseTwistStamped)
        self._trajectory_pub = yield self._node_handle.advertise('trajectory', PoseTwistStamped)

        self._dvl_range_sub = yield self._node_handle.subscribe('dvl/range', Float64Stamped)
        self._tf_listener = yield tf.TransformListener(self._node_handle)
        defer.returnValue(self)

    @property
    def pose(self):
        # @forrest: Why trajectory -> last message and not odom??
        # last_pose_msg = self._trajectory_sub.get_last_message()
        # pose = orientation_helpers.PoseEditor.from_PoseTwistStamped(last_pose_msg)
        last_odom_msg = self._odom_sub.get_last_message()
        pose = orientation_helpers.PoseEditor.from_Odometry(last_odom_msg)
        return pose

    @util.cancellableInlineCallbacks
    def last_pose(self):
        last_pose = yield self._odom_sub.get_next_message()
        defer.returnValue(last_pose)

    @property
    def move(self):
        return _PoseProxy(self, self.pose)

    @util.cancellableInlineCallbacks
    def get_dvl_range(self):
        msg = yield self._dvl_range_sub.get_next_message()
        defer.returnValue(msg.data)


_subs = {}


@util.cancellableInlineCallbacks
def get_sub(node_handle, need_trajectory=True):
    if node_handle not in _subs:
        _subs[node_handle] = None  # placeholder to prevent this from happening reentrantly
        _subs[node_handle] = yield _Sub(node_handle)._init()
        # XXX remove on nodehandle shutdown
    defer.returnValue(_subs[node_handle])
