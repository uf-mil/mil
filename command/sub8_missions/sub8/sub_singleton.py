#!/usr/bin/env python
from __future__ import division

import genpy
from txros import action, util, tf, serviceclient
import rospkg

from mil_msgs.msg import MoveToAction, PoseTwistStamped, RangeStamped
from sub8 import pose_editor
import mil_ros_tools
from sub8_msgs.srv import VisionRequest, VisionRequestRequest, VisionRequest2DRequest, VisionRequest2D
from mil_msgs.srv import SetGeometry, SetGeometryRequest
from mil_msgs.srv import ObjectDBQuery, ObjectDBQueryRequest
from sub8_msgs.srv import SetValve, SetValveRequest
from std_srvs.srv import SetBool, SetBoolRequest, Trigger, TriggerRequest
from nav_msgs.msg import Odometry
from tf.transformations import quaternion_multiply, quaternion_from_euler
from sensor_msgs.msg import PointCloud2
import sensor_msgs.point_cloud2 as pc2

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
        self._get_2d_service = nh.get_service_client(service_root + "/2D",
                                                     VisionRequest2D)
        self._get_pose_service = nh.get_service_client(service_root + "/pose",
                                                       VisionRequest)
        self._enable_service = nh.get_service_client(service_root + "/enable",
                                                     SetBool)
        self._set_geometry_service = nh.get_service_client(
            service_root + "/set_geometry", SetGeometry)

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
            pose = self._get_2d_service(
                VisionRequest2DRequest(target_name=target))
        except (serviceclient.ServiceError):
            return None
        return pose

    def get_pose(self, target='', in_frame=None):
        '''Get the 3D pose of the object we're after
        TODO:
            - Implement in_frame
            - Use the time information in the header
        '''
        try:
            pose = self._get_pose_service(
                VisionRequestRequest(target_name=target))
        except (serviceclient.ServiceError):
            return None
        except Exception, e:
            print type(e)
        return pose

    def set_geometry(self, polygon):
        try:
            res = self._set_geometry_service(SetGeometryRequest(model=polygon))
        except (serviceclient.ServiceError):
            return None
        return res

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
        config_file = os.path.join(
            rospack.get_path('sub8_missions'), 'sub8', file_name)
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
            return _PoseProxy(
                self._sub,
                getattr(self._pose, name)(*args, **kwargs),
                print_only=self.print_only)

        return sub_attr_proxy

    # Some special moves
    def to_height(self, height):
        dist_to_bot = self._sub._dvl_range_sub.get_last_message()
        delta_height = dist_to_bot.range - height
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

        goal = self._sub._moveto_action_client.send_goal(
            self._pose.as_MoveToGoal(*args, **kwargs))
        return goal.get_result()

    def go_trajectory(self, *args, **kwargs):
        traj = self._sub._trajectory_pub.publish(
            self._pose.as_PoseTwistStamped(*args, **kwargs))
        return traj


class _ActuatorProxy(object):
    '''
    Wrapper for making service calls to pneumatic valve board.

    Example usage:
    yield self.sub.gripper_open()
    yield self.sub.set('torpedo1', True)
    '''

    def __init__(self, nh, namespace='actuator_driver'):
        self._actuator_service = nh.get_service_client(namespace + '/actuate',
                                                       SetValve)
        self._raw_service = nh.get_service_client(namespace + '/actuate_raw',
                                                  SetValve)

    def open(self, name):
        return self.set(name, False)

    def close(self, name):
        return self.set(name, True)

    def set(self, name, opened):
        return self._actuator_service(
            SetValveRequest(actuator=name, opened=opened))

    def set_raw(self, name, opened):
        return self._raw_service(SetValveRequest(actuator=name, opened=opened))

    def gripper_open(self):
        return self.set('gripper', True)

    def gripper_close(self):
        return self.set('gripper', False)

    def shoot_torpedo1(self):
        return self.set('torpedo1', True)

    def shoot_torpedo2(self):
        return self.set('torpedo2', True)

    def drop_marker(self):
        return self.set('dropper', True)


class _Sub(object):
    def __init__(self, node_handle):
        self.nh = node_handle
        self.test_mode = False

    @util.cancellableInlineCallbacks
    def _init(self):
        self._moveto_action_client = yield action.ActionClient(
            self.nh, 'moveto', MoveToAction)
        self._odom_sub = yield self.nh.subscribe('odom', Odometry)
        self._trajectory_sub = yield self.nh.subscribe('trajectory',
                                                       PoseTwistStamped)
        self._trajectory_pub = yield self.nh.advertise('trajectory',
                                                       PoseTwistStamped)
        self._dvl_range_sub = yield self.nh.subscribe('dvl/range',
                                                      RangeStamped)
        self._tf_listener = yield tf.TransformListener(self.nh)

        self.vision_proxies = _VisionProxies(self.nh, 'vision_proxies.yaml')
        self.actuators = _ActuatorProxy(self.nh)

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
            blank = mil_ros_tools.pose_to_numpy(Odometry().pose.pose)
            defer.returnValue(blank)

        next_odom_msg = yield self._odom_sub.get_next_message()
        pose = mil_ros_tools.pose_to_numpy(next_odom_msg.pose.pose)
        defer.returnValue(pose)

    @property
    def move(self):
        return _PoseProxy(self, self.pose, self.test_mode)

    @util.cancellableInlineCallbacks
    def get_dvl_range(self):
        msg = yield self._dvl_range_sub.get_next_message()
        defer.returnValue(msg.range)

    @util.cancellableInlineCallbacks
    def get_in_frame(self, pose_stamped, frame='/map'):
        '''TODO'''
        transform = yield self._tf_listener.get_transform(
            frame, pose_stamped.header.frame_id, pose_stamped.header.stamp)
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
        searcher = self._run_search_pattern(loop, speed).addErrback(
            self.catch_error)

        start_pose = self.sub.move.forward(0)
        start_time = self.sub.nh.get_time()
        while self.sub.nh.get_time() - start_time < genpy.Duration(timeout):

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
                    yield self.sub.move.relative(
                        np.array(pose)).go(speed=speed)
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
                print "SEARCHER - Object found! {}/{}".format(
                    spotings + 1, spotings_req)
                spotings += 1
                if spotings >= spotings_req:
                    self.object_found = True
                    self.response = resp
                    break
            else:
                spotings = 0

            yield self.sub.nh.sleep(.5)


class PoseSequenceCommander(object):
    def __init__(self, sub):
        self.sub = sub

    @util.cancellableInlineCallbacks
    def go_to_sequence_eulers(self, positions, orientations, speed=0.2):
        '''Pass a list of positions and orientations (euler).
        Each is realive to the sub's pose folloing the previous
        pose command.
        '''
        for i in xrange(len(positions)):
            yield self.sub.move.look_at_without_pitching(
                np.array(positions[i][0:3])).go(speed)
            yield self.sub.move.relative(np.array(positions[i][0:3])).go(speed)
            yield self.sub.move.set_orientation(
                quaternion_multiply(self.sub.pose.orientation,
                                    quaternion_from_euler(
                                        orientations[i][0], orientations[i][1],
                                        orientations[i][2]))).go(speed)

    @util.cancellableInlineCallbacks
    def go_to_sequence_quaternions(self, positions, orientations, speed=0.2):
        '''Pass a list of positions and orientations (quaternion).
        Each is realive to the sub's pose folloing the previous
        pose command.
        '''
        for i in xrange(len(positions)):
            yield self.sub.move.look_at_without_pitching(
                np.array(positions[i][0:3])).go(speed)
            yield self.sub.move.relative(np.array(positions[i][0:3])).go(speed)
            yield self.sub.move.set_orientation(
                quaternion_multiply(
                    self.sub.pose.orientation,
                    (orientations[i][0], orientations[i][1],
                     orientations[i][2], orientations[i][3]))).go(speed)


class SonarObjects(object):
    def __init__(self, sub, pattern=None):
        """
        SonarObjects: a helper to search and find objects

        Everything must be in map frame

        Parameters:
        sub: the sub object
        pattern: an array of pose goals (i.e: [sub.move.forward(1)])
        """
        self.sub = sub
        if pattern is None:
            self.pattern = [sub.move.forward(0)]
        self.pattern = pattern
        self._clear_pcl = self.sub.nh.get_service_client(
            '/ogrid_pointcloud/clear_pcl', Trigger)

        self._objects_service = self.sub.nh.get_service_client(
            '/ogrid_pointcloud/get_objects', ObjectDBQuery)

    def __del__(self):
        print('cleared SonarObject -- thanks TX')

    @util.cancellableInlineCallbacks
    def start_search(self, speed=0.5, clear=False):
        """
        Do a search and return all objects

        Parameters:
        speed: how fast sub should move
        clear: clear pointcloud
        """
        if clear:
            print 'SONAR_OBJECTS: clearing pointcloud'
            self._clear_pcl(TriggerRequest())

        print 'SONAR_OBJECTS: running pattern'
        yield self._run_pattern(speed)

        print 'SONAR_OBJECTS: requesting objects'
        res = yield self._objects_service(ObjectDBQueryRequest())
        defer.returnValue(res)

    @util.cancellableInlineCallbacks
    def start_search_in_cone(self,
                             start_point,
                             ray,
                             angle_tol=30,
                             distance_tol=10,
                             speed=0.5,
                             clear=False,
                             c_func=None):
        if clear:
            print 'SONAR_OBJECTS: clearing pointcloud'
            self._clear_pcl(TriggerRequest())

        yield self.sub.nh.sleep(1)

        for pose in self.pattern:
            yield pose.go(speed=speed, blind=True)
            # sleep
            yield self.sub.nh.sleep(0.1)

            # Break out of loop if we find something satisifying function
            res = yield self._objects_service(ObjectDBQueryRequest())
            g_obj = self._get_objects_within_cone(res.objects, start_point,
                                                  ray, angle_tol, distance_tol)
            g_obj = self._sort_by_angle(g_obj, ray, start_point)
            yield

            if c_func is not None:
                out = c_func(g_obj, ray)
                print 'SONAR_OBJECTS: ' + str(out)
                if out is not None or out is True:
                    print 'SONAR_OBJECTS: found objects satisfing function'
                    break

        res = yield self._objects_service(ObjectDBQueryRequest())
        g_obj = self._get_objects_within_cone(res.objects, start_point, ray,
                                              angle_tol, distance_tol)
        g_obj = self._sort_by_angle(g_obj, ray, start_point)
        res.objects = g_obj
        defer.returnValue(res)

    @util.cancellableInlineCallbacks
    def start_until_found_x(self, speed=0.5, clear=False, object_count=0):
        """
        Search until a number of objects are found

        Parameters:
        speed: how fast sub should move
        clear: clear pointcloud
        object_count: how many objects we want
        """
        if clear:
            print 'SONAR_OBJECTS: clearing pointcloud'
            self._clear_pcl(TriggerRequest())
        count = -1
        while count < object_count:
            for pose in self.pattern:
                yield pose.go(speed=speed)
                res = yield self._objects_service(ObjectDBQueryRequest())
                count = len(res.objects)
                if count >= object_count:
                    defer.returnValue(res)
        defer.returnValue(None)

    @util.cancellableInlineCallbacks
    def start_until_found_in_cone(self,
                                  start_point,
                                  speed=0.5,
                                  clear=False,
                                  object_count=0,
                                  ray=np.array([0, 1, 0]),
                                  angle_tol=30,
                                  distance_tol=12):
        """
        Search until objects are found within a cone-shaped range

        Parameters:
        start_point: numpy array for the starting point of the direction vector
        speed: how fast the sub should move
        clear: should the pointcloud be clear beforehand
        object_count: how many objects we are looking for
        ray: the direction vector
        angle_tol: how far off the direction vector should be allowed
        distance_tol: how far away are we willing to accept

        Returns:
        ObjectDBQuery: with objects field filled by good objects
        """
        if clear:
            print 'SONAR_OBJECTS: clearing pointcloud'
            self._clear_pcl(TriggerRequest())
        count = -1
        while count < object_count:
            for pose in self.pattern:
                yield pose.go(speed=speed, blind=True)
                res = yield self._objects_service(ObjectDBQueryRequest())
                g_obj = self._get_objects_within_cone(
                    res.objects, start_point, ray, angle_tol, distance_tol)
                if g_obj is None:
                    continue
                count = len(g_obj)
                print 'SONAR OBJECTS: found {} that satisfy cone'.format(count)
                if count >= object_count:
                    g_obj = self._sort_by_angle(g_obj, ray, start_point)
                    res.objects = g_obj
                    defer.returnValue(res)
        defer.returnValue(None)

    @staticmethod
    def _get_objects_within_cone(objects, start_point, ray, angle_tol,
                                 distance_tol):
        ray = ray / np.linalg.norm(ray)
        out = []
        for o in objects:
            print '=' * 50
            pos = mil_ros_tools.rosmsg_to_numpy(o.pose.position)
            print 'pos {}'.format(pos)
            dist = np.dot(pos - start_point, ray)
            print 'dist {}'.format(dist)
            if dist > distance_tol or dist < 0:
                continue
            vec_for_pos = pos - start_point
            vec_for_pos = vec_for_pos / np.linalg.norm(vec_for_pos)
            angle = np.arccos(vec_for_pos.dot(ray)) * 180 / np.pi
            print 'angle {}'.format(angle)
            if angle > angle_tol:
                continue
            out.append(o)
        return out

    @staticmethod
    def _sort_by_angle(objects, ray, start_point):
        """
        _sort_by_angle: returns object list sorted by angle

        Parameters:
        objects:
        ray: directional unit vector
        start_point: base point for vector in map
        """
        positions = [
            mil_ros_tools.rosmsg_to_numpy(o.pose.position) for o in objects
        ]
        dots = [(p / np.linalg.norm(p) - start_point).dot(ray)
                for p in positions]
        idx = np.argsort(dots)
        return np.array(objects)[idx]

    @util.cancellableInlineCallbacks
    def _run_pattern(self, speed):
        for pose in self.pattern:
            yield pose.go(speed=speed)


class SonarPointcloud(object):
    def __init__(self, sub, pattern=None):
        if pattern is None:
            pattern = [sub.move.zero_roll_and_pitch()
                       ] + [sub.move.pitch_down_deg(5)] * 5 + [
                           sub.move.zero_roll_and_pitch()
                       ]
        self.sub = sub
        self.pointcloud = None
        self.pattern = pattern

    @util.cancellableInlineCallbacks
    def start(self, speed=0.2):
        self._plane_subscriber = yield self.sub.nh.subscribe(
            '/ogrid_pointcloud/point_cloud/plane', PointCloud2)
        yield self._run_move_pattern(speed)

        pc_gen = np.asarray(
            list(
                pc2.read_points(
                    self.pointcloud,
                    skip_nans=True,
                    field_names=('x', 'y', 'z'))))
        defer.returnValue(pc_gen)

    @util.cancellableInlineCallbacks
    def _cat_newcloud(self):
        data = yield self._plane_subscriber.get_next_message()
        if self.pointcloud is None:
            self.pointcloud = data
        else:
            gen = list(
                pc2.read_points(
                    data, skip_nans=True, field_names=('x', 'y', 'z')))
            pc_gen = list(
                pc2.read_points(
                    self.pointcloud,
                    skip_nans=True,
                    field_names=('x', 'y', 'z')))
            concat = np.asarray(gen + pc_gen, np.float32)
            print 'SONAR_POINTCLOUD - current size: {}'.format(concat.shape)
            self.pointcloud = mil_ros_tools.numpy_to_pointcloud2(concat)
        yield

    @util.cancellableInlineCallbacks
    def _run_move_pattern(self, speed):
        for pose in self.pattern:
            if type(pose) == list or type(pose) == np.ndarray:
                yield self.sub.move.relative(np.array(pose)).go(speed=speed)
            else:
                yield pose.go()
            yield self._cat_newcloud()

            yield self.sub.nh.sleep(2)


_subs = {}


@util.cancellableInlineCallbacks
def get_sub(node_handle, need_trajectory=True):
    if node_handle not in _subs:
        _subs[
            node_handle] = None  # placeholder to prevent this from happening reentrantly
        _subs[node_handle] = yield _Sub(node_handle)._init()
        # XXX remove on nodehandle shutdown
    defer.returnValue(_subs[node_handle])
