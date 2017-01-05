#!/usr/bin/env python
from __future__ import division

from txros import action, util, tf, serviceclient
import rospkg
from tf import transformations

from uf_common.msg import MoveToAction, PoseTwistStamped, Float64Stamped
from sub8 import pose_editor
import sub8_tools
from sub8_msgs.srv import VisionRequest, VisionRequestRequest, VisionRequest2DRequest, VisionRequest2D
from std_srvs.srv import SetBool, SetBoolRequest
from nav_msgs.msg import Odometry

import numpy as np
from twisted.internet import defer
import os
import yaml


class VisionProxy(object):
    '''General Interface for communicating with perception nodes.
    Make sure your perception nodes are structured as follows:
        - The node provides an enable service to start and stop percetion
        - The node provides a 3d and/or 2d pose service to get the position of the
            object of intrest in real world or pixel
        - All services need to have the same root, for example: 
            A buoy finder node may provide the following:
            /vision/buoy_finder/enable  # Starts and stops the perception (type = setBool)
            /vision/buoy_finder/2D      # Returns the pixel coordinates for the buoy (type = VisionRequest2D)
            /vision/buoy_finder/pose    # Returns a 3d pose of the buoy (type = VisionRequest)
    '''
    def __init__(self, service_root, nh):
        assert 'vision' in service_root, "expected 'vision' in the name of service_root"
        self._get_2d_service = nh.get_service_client(service_root + "/2D", VisionRequest2D)
        self._get_pose_service = nh.get_service_client(service_root + "/pose", VisionRequest)
        self._enable_service = nh.get_service_client(service_root + "/enable", SetBool)
    
    def start(self):
        '''Allow user to start the vision processing backend
        Can be used when the mission starts
        '''
        return self._enable_service(SetBoolRequest(data=True))

    def stop(self):
        '''Allow user to stop the vision processing backend
        Can be used after the mission completes
        '''
        return self._enable_service(SetBoolRequest(data=False))

    def get_2d(self, target=''):
        '''Get the 2D projection of the thing
        TODO: Do something intelligent with the stamp
            - This is not "obviously" in any reference frame
            - We'll assume the user knows what they're doing
            - Determine rotation around Z and undo that?

        Camera deprojection stuff should be done elsewhere, this function is for when
            we don't know the depth of the thing we're targeting
        '''
        try:
            pose = self._get_2d_service(VisionRequest2DRequest(target_name=target))
        except(serviceclient.ServiceError):
            return None
        return pose

    def get_pose(self, target='', in_frame=None):
        '''Get the 3D pose of the object we're after
        TODO:
            - Implement in_frame
            - Use the time information in the header
        '''
        try:
            pose = self._get_pose_service(VisionRequestRequest(target_name=target))
        except(serviceclient.ServiceError):
            return None
        except Exception, e:
            print type(e)
        return pose

    @classmethod
    def get_response_direction(self, vision_response):
        xy = np.array([vision_response.pose.x, vision_response.pose.y])
        bounds = np.array([[vision_response.max_x, vision_response.max_y]])
        theta = vision_response.pose.theta
        return (xy - (bounds / 2.0)) / bounds, theta

class _VisionProxies(object):
    '''Gives interface to vision proxies.
    Add vision proxy names and roots to the yaml config file and acccess them through here, ex:

        >>> yield sub.vision_proxies.buoy_finder.start()
        >>> pose_2d = yield sub.vision_proxies.buoy_finder.get_2D('red')

    '''
    def __init__(self, nh, file_name):
        rospack = rospkg.RosPack()
        config_file = os.path.join(rospack.get_path('sub8_missions'), 'sub8', file_name)
        f = yaml.load(open(config_file, 'r'))

        self.proxies = {}
        for name, params in f.items():
            self.proxies[name] = VisionProxy(params['root'], nh)

    def __getattr__(self, proxy):
        return self.proxies.get(proxy, None)


class _PoseProxy(object):
    def __init__(self, sub, pose, print_only=False):
        self._sub = sub
        self._pose = pose
        self.print_only = print_only
    
    # Normal moves get routed here
    def __getattr__(self, name):
        def sub_attr_proxy(*args, **kwargs):
            return _PoseProxy(self._sub, getattr(self._pose, name)(*args, **kwargs), print_only=self.print_only)
        return sub_attr_proxy

    # Some special moves
    def to_height(self, height):
        dist_to_bot = self._sub._dvl_range_sub.get_last_message()
        delta_height = dist_to_bot.data - height
        return self.down(delta_height)

    def check_goal(self):
        '''Check end goal for feasibility.
        Current checks are:
            - End goal can't be above the water
        '''
        # End goal can't be above the water
        if self._pose.position[2] > 0:
            print "GOAL TOO HIGH"
            self._pos.position = -0.6

    def go(self, *args, **kwargs):
        if self.print_only:
            print self._pose
            return self._sub.nh.sleep(0.1)

        self.check_goal()

        goal = self._sub._moveto_action_client.send_goal(self._pose.as_MoveToGoal(*args, **kwargs))
        return goal.get_result()

    def go_trajectory(self, *args, **kwargs):
        traj = self._sub._trajectory_pub.publish(
            self._pose.as_PoseTwistStamped(*args, **kwargs)
        )
        return traj


class _Sub(object):
    def __init__(self, node_handle):
        self.nh = node_handle
        self.test_mode = False

    @util.cancellableInlineCallbacks
    def _init(self):
        self._moveto_action_client = yield action.ActionClient(self.nh, 'moveto', MoveToAction)
        self._odom_sub = yield self.nh.subscribe('odom', Odometry)
        self._trajectory_sub = yield self.nh.subscribe('trajectory', PoseTwistStamped)
        self._trajectory_pub = yield self.nh.advertise('trajectory', PoseTwistStamped)
        self._dvl_range_sub = yield self.nh.subscribe('dvl/range', Float64Stamped)
        self._tf_listener = yield tf.TransformListener(self.nh)
        
        self.vision_proxies = _VisionProxies(self.nh, 'vision_proxies.yaml')

        defer.returnValue(self)

    def set_test_mode(self):
        self.test_mode = True

    @property
    def pose(self):
        last_odom_msg = self._odom_sub.get_last_message()
        if self.test_mode:
            last_odom_msg = Odometry()  # All 0's
        pose = pose_editor.PoseEditor.from_Odometry(last_odom_msg)
        return pose

    @util.cancellableInlineCallbacks
    def tx_pose(self):
        '''Slighty safer to use.'''
        if self.test_mode:
            yield self.nh.sleep(.1)
            blank = sub8_tools.pose_to_numpy(Odometry().pose.pose)
            defer.returnValue(blank)

        next_odom_msg = yield self._odom_sub.get_next_message()
        pose = sub8_tools.pose_to_numpy(next_odom_msg.pose.pose)
        defer.returnValue(pose)

    @property
    def move(self):
        return _PoseProxy(self, self.pose, self.test_mode)

    @util.cancellableInlineCallbacks
    def get_dvl_range(self):
        msg = yield self._dvl_range_sub.get_next_message()
        defer.returnValue(msg.data)

    @util.cancellableInlineCallbacks
    def get_in_frame(self, pose_stamped, frame='/map'):
        '''TODO'''
        transform = yield self._tf_listener.get_transform(
            frame,
            pose_stamped.header.frame_id,
            pose_stamped.header.stamp
        )
        tft = tf.Transform.from_Pose_message(pose_stamped.pose)
        full_transform = transform * tft
        position = np.array(full_transform._p)
        orientation = np.array(full_transform._q)

        defer.returnValue([position, orientation])


class Searcher(object):
    def __init__(self, sub, vision_proxy, search_pattern):
        '''
        Give a sub_singleton, the a function to call for the object you're looking for, and a list poses to execute in
            order to find it (can be a list of relative positions or pose_editor poses).
        '''
        self.sub = sub
        self.vision_proxy = vision_proxy
        self.search_pattern = search_pattern

        self.object_found = False
        self.response = None

    def catch_error(self, failure):
        if failure.check(defer.CancelledError):
            print "SEARCHER - Cancelling defer."
        else:
            print "SEARCHER - There was an error."
            print failure.printTraceback()
            # Handle error

    @util.cancellableInlineCallbacks
    def start_search(self, timeout=60, loop=True, spotings_req=2, speed=.1):
        print "SERACHER - Starting."
        looker = self._run_look(spotings_req).addErrback(self.catch_error)
        searcher = self._run_search_pattern(loop, speed).addErrback(self.catch_error)

        start_pose = self.sub.move.forward(0)
        start_time = self.sub.nh.get_time()
        while self.sub.nh.get_time() - start_time < txros.genpy.Duration(timeout):

            # If we find the object
            if self.object_found:
                searcher.cancel()
                print "SEARCHER - Object found."
                defer.returnValue(self.response)

            yield self.sub.nh.sleep(0.1)

        print "SEARCHER - Object NOT found. Returning to start position."
        looker.cancel()
        searcher.cancel()

        yield start_pose.go()

    @util.cancellableInlineCallbacks
    def _run_search_pattern(self, loop, speed):
        '''
        Look around using the search pattern.
        If `loop` is true, then keep iterating over the list until timeout is reached or we find it.
        '''
        print "SEARCHER - Executing search pattern."
        if loop:
            while True:
                for pose in self.search_pattern:
                    print "SEARCHER - going to next position."
                    if type(pose) == list or type(pose) == np.ndarray:
                        yield self.sub.move.relative(pose).go(speed=speed)
                    else:
                        yield pose.go()

                    yield self.sub.nh.sleep(2)

        else:
            for pose in self.search_pattern:
                if type(pose) == list or type(pose) == np.ndarray:
                    yield self.sub.move.relative(np.array(pose)).go(speed=speed)
                else:
                    yield pose.go()

                yield self.sub.nh.sleep(2)

    @util.cancellableInlineCallbacks
    def _run_look(self, spotings_req):
        '''
        Look for the object using the vision proxy.
        Only return true when we spotted the objects `spotings_req` many times (for false positives).
        '''
        spotings = 0
        print "SERACHER - Looking for object."
        while True:
            resp = yield self.vision_proxy()
            if resp.found:
                print "SEARCHER - Object found! {}/{}".format(spotings + 1, spotings_req)
                spotings += 1
                if spotings >= spotings_req:
                    self.object_found = True
                    self.response = resp
                    break
            else:
                spotings = 0

            yield self.sub.nh.sleep(.5)


_subs = {}

@util.cancellableInlineCallbacks
def get_sub(node_handle, need_trajectory=True):
    if node_handle not in _subs:
        _subs[node_handle] = None  # placeholder to prevent this from happening reentrantly
        _subs[node_handle] = yield _Sub(node_handle)._init()
        # XXX remove on nodehandle shutdown
    defer.returnValue(_subs[node_handle])
