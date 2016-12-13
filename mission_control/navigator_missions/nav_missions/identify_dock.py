#!/usr/bin/env python
from __future__ import division
import txros
import std_srvs.srv
import numpy as np
import tf
import tf.transformations as trns
from geometry_msgs.msg import Point
from std_srvs.srv import SetBool, SetBoolRequest
from twisted.internet import defer
from visualization_msgs.msg import Marker,MarkerArray
import navigator_tools
from navigator_tools import fprint, MissingPerceptionObject
from navigator_msgs.srv import GetDockBays, GetDockBaysRequest
from navigator_msgs.srv import CameraToLidarTransform,CameraToLidarTransformRequest
from docking_ogrid_manager import OgridFactory

print_good = lambda message: fprint(message, title="IDENTIFY DOCK",  msg_color='green')
print_bad  = lambda message: fprint(message, title="IDENTIFY DOCK",  msg_color='yellow')

class IdentifyDockMission:
    CIRCLE_RADIUS      = 16  # Radius of circle in dock circle pattern
    BAY_SEARCH_TIMEOUT = 80  # Seconds to circle dock looking for bays before giving up
    LOOK_AT_DISTANCE   = 10  # Distance in front of bay to move to to look at shape
    LOOK_SHAPE_TIMEOUT = 10  # Time to look at bay with cam to see symbol
    DOCK_DISTANCE      = 3   # Distance in front of bay point to dock to
    DOCK_SLEEP_TIME    = 5   # Time to wait after docking before undocking aka show off time 1.016+
    WAYPOINT_NAME      = "dock"
    TIMEOUT_CIRCLE_VISION_ONLY    = 75

    def __init__(self, navigator):
        self.navigator = navigator

    def _init_default(self):
        self.ogrid_activation_client = self.navigator.nh.get_service_client('/identify_dock/active', SetBool)
        self.get_bays = self.navigator.nh.get_service_client('/identify_dock/get_bays', GetDockBays)
        self.bays_res = None
        self.bays_symbols = (None, None, None)
        self.docked = (False, False)

    def start_ogrid(self):
        pass
        #return self.ogrid_activation_client(SetBoolRequest(data=True))

    def stop_ogrid(self):
        pass
        #return self.ogrid_activation_client(SetBoolRequest(data=False))

    @txros.util.cancellableInlineCallbacks
    def get_target_bays(self):
        bay_1_color = yield self.navigator.mission_params["dock_color_1"].get()
        bay_1_shape = yield self.navigator.mission_params["dock_shape_1"].get()
        bay_2_color = yield self.navigator.mission_params["dock_color_2"].get()
        bay_2_shape = yield self.navigator.mission_params["dock_shape_2"].get()
        self.bay_1 = (bay_1_shape, bay_1_color)
        self.bay_2 = (bay_2_shape, bay_2_color)
        print_good("Docking in Bay (Shape={}, Color={}) then (Shape={}, Color={})".format(bay_1_shape, bay_1_color, bay_2_shape, bay_2_color))

    @txros.util.cancellableInlineCallbacks
    def get_waypoint(self):
        res = yield self.navigator.database_query(self.WAYPOINT_NAME)
        self.dock_pose = navigator_tools.rosmsg_to_numpy(res.objects[0].position)

    @txros.util.cancellableInlineCallbacks
    def search_bays(self):
        res = yield self.get_bays(GetDockBaysRequest())
        if not res.success:
            print_bad("Bays Not found Error={}".format(res.error))
        else:
            self.bays_res = res
            self.bay_poses = (navigator_tools.rosmsg_to_numpy(res.bays[0]),
              navigator_tools.rosmsg_to_numpy(res.bays[1]),
              navigator_tools.rosmsg_to_numpy(res.bays[2]))
            self.bay_normal = navigator_tools.rosmsg_to_numpy(res.normal)
        defer.returnValue(res.success)

    @txros.util.cancellableInlineCallbacks
    def circle_dock(self):
        '''
        Circle the dock until the bays are identified by the perception service
        '''
        print_good("Starting circle search")
        pattern = self.navigator.move.d_circle_point(self.dock_pose, radius=self.CIRCLE_RADIUS)
        yield next(pattern).go()
        searcher = self.navigator.search(search_pattern=pattern, looker=self.search_bays)
        yield searcher.start_search(timeout=self.BAY_SEARCH_TIMEOUT, move_type="skid", loop=True)
        if self.bays_res == None:
            raise Exception("Bays not found after circling")
        print_good("Ended circle search")

    @txros.util.cancellableInlineCallbacks
    def search_shape(self):
        res = self.navigator.vision_proxies["get_shape_front"].get_response(Shape="ANY", Color="ANY")
        if res.found:
            self.bays_symbols[self.check_index] = res
        defer.returnValue(res.found)

    @txros.util.cancellableInlineCallbacks
    def dock_in_bay(self, index):
        move_front = self.navigator.move.set_position(self.bay_poses[i] + self.bay_normal * self.LOOK_AT_DISTANCE).look_at(self.bay_poses[i])
        move_in    = self.navigator.move.set_position(self.bay_poses[i] + self.bay_normal *  self.DOCK_DISTANCE).look_at(self.bay_poses[i])
        yield move_front.go()
        yield move_in.go()
        yield self.navigator.nh.sleep(DOCK_SLEEP_TIME)
        yield move_front.go()
        
    @txros.util.cancellableInlineCallbacks
    def dock_if_identified(self):
        '''
        See if a bay was identified with desired symbol, dock if so
        '''
        for i, shape_res in enumerate(self.bays_symbols):
            if not shape_res == None:
                if self.correct_shape(self.bay_1, (shape_res.Shape, shape_res.Color)) and (not self.docked[0]):
                    print_good("Bay {index} (Shape={res.Shape}, Color={res.Color}) has first search bay, docking".format(index=i, res=shape_res))
                    yield self.dock_in_bay(i)
                    self.docked[0] = True
                elif self.correct_shape(self.bay_2, (shape_res.Shape, shape_res.Color)) and (not self.docked[1]) and (self.docked[0]):
                    print_good("Bay {index} (Shape={res.Shape}, Color={res.Color}) has seconds search bay, docking".format(index=i, res=shape_res))
                    yield self.dock_in_bay(i)
                    self.docked[1] = True
                else:
                    print_bad("Bay {index} (Shape={res.Shape}, Color={res.Color}) does not match a search shape".format(index=i, res=shape_res))
                  
            

    def correct_shape(self, (desired_shape, desired_color), (test_shape, test_color) ):
        return (desired_color == "ANY" or desired_color == test_color) and (desired_shape == "ANY" or desired_shape == test_shape)

    @txros.util.cancellableInlineCallbacks
    def check_bays(self):
        '''
        Go in front of each bay and look at the symbols present
        '''
        for i, pose in enumerate(self.bay_poses):
            self.check_index = i
            move = self.navigator.move.set_position(pose + self.bay_normal * self.LOOK_AT_DISTANCE).look_at(pose)
            yield move.go()
            searcher = self.navigator.search(looker=self.search_shape)
            yield searcher.start_search(timeout=self.LOOK_SHAPE_TIMEOUT)
            yield self.dock_if_identified()

    @txros.util.cancellableInlineCallbacks
    def find_and_dock(self):
        self._init_default()
        yield self.navigator.vision_proxies["get_shape_front"].start()
        yield self.get_target_bays()
        yield self.get_waypoint()
        yield self.start_ogrid()
        yield self.circle_dock()
        yield self.stop_ogrid()
        yield self.navigator.vision_proxies["get_shape_front"].stop()



    '''
    Alternative method using vision only like detect deliver, no lidar analysis perception
    '''
    @txros.util.cancellableInlineCallbacks
    def _init_vision_only(self):
        self.ogrid_activation_client = self.navigator.nh.get_service_client('/identify_dock/active', SetBool)
        self.cameraLidarTransformer = self.navigator.nh.get_service_client("/camera_to_lidar/stereo_right_cam", CameraToLidarTransform)
        self.ogrid_clear = OgridFactory(self.navigator)
        yield self.ogrid_clear.init_()
        self.identified_shapes = {}

    def update_shape(self, shape_res, normal_res, tf):
       print_good("Found (Shape={}, Color={} in a bay".format(shape_res.Shape, shape_res.Color))
       self.identified_shapes[(shape_res.Shape, shape_res.Color)] = self.get_shape_pos(normal_res, tf)

    def done_circling(self):
        print_good("CURRENT IDENTIFIED BAYS: {}".format(self.identified_shapes))
        for shape_color, point_normal in self.identified_shapes.iteritems():
            if self.correct_shape(self.bay_1, shape_color):
              for shape_color2, point_normal in self.identified_shapes.iteritems():
                  if self.correct_shape(self.bay_2, shape_color2):
                      return True
        return False

    @txros.util.cancellableInlineCallbacks
    def search_shape_vision_only(self):
        shapes = yield self.navigator.vision_proxies["get_shape_front"].get_response(Shape="ANY", Color="ANY")
        if shapes.found:
            for shape in shapes.shapes.list:
                normal_res = yield self.get_normal(shape)
                if normal_res.success:
                    enu_cam_tf = yield self.navigator.tf_listener.get_transform('/enu', '/'+shape.header.frame_id, shape.header.stamp)
                    self.update_shape(shape, normal_res, enu_cam_tf)
                    if (self.done_circling()):
                        defer.returnValue(True)
                else:
                      print_bad("NORMAL ERROR: {}".format(normal_res.error))
        else:
            print_bad("SHAPES ERROR: {}".format(shapes.error))
        defer.returnValue(False)

    def _bounding_rect(self,points):
        np_points = map(navigator_tools.point_to_numpy, points)
        xy_max = np.max(np_points, axis=0)
        xy_min = np.min(np_points, axis=0)
        return np.append(xy_max, xy_min)

    @txros.util.cancellableInlineCallbacks
    def circle_dock_vision_only(self):
        print_good("Starting circle search (vision only)")
        pattern = self.navigator.move.d_circle_point(self.dock_pose, radius=self.CIRCLE_RADIUS)
        searcher = self.navigator.search(search_pattern=pattern, looker=self.search_shape_vision_only)
        yield searcher.start_search(timeout=self.TIMEOUT_CIRCLE_VISION_ONLY, spotings_req=1, move_type="skid")
        print_good("Ended circle search")

    def normal_is_sane(self, vector3):
         return abs(navigator_tools.rosmsg_to_numpy(vector3)[1]) < 0.4

    @txros.util.cancellableInlineCallbacks
    def get_normal(self, shape):
        req = CameraToLidarTransformRequest()
        req.header = shape.header
        req.point = Point()
        req.point.x = shape.CenterX
        req.point.y = shape.CenterY
        rect = self._bounding_rect(shape.points)
        req.tolerance = int(min(rect[0]-rect[3],rect[1]-rect[4])/2.0)
        normal_res = yield self.cameraLidarTransformer(req)
        if not self.normal_is_sane(normal_res.normal):
            normal_res.success = False
            print_bad("UNREASONABLE NORMAL={}".format(normal_res.normal))
            normal_res.error = "UNREASONABLE NORMAL"
        defer.returnValue(normal_res)

    def get_shape_pos(self, normal_res, enu_cam_tf):
        enunormal = enu_cam_tf.transform_vector(navigator_tools.rosmsg_to_numpy(normal_res.normal))
        enupoint = enu_cam_tf.transform_point(navigator_tools.rosmsg_to_numpy(normal_res.closest))
        return (enupoint, enunormal)

    @txros.util.cancellableInlineCallbacks
    def dock_vision_only(self):
        bay_1_shape = None
        bay_2_shape = None
        incorrect_shapes = []
        for shape_color, point_normal in self.identified_shapes.iteritems():
            if   self.correct_shape(self.bay_1, shape_color):
                bay_1_shape = point_normal
            elif self.correct_shape(self.bay_2, shape_color):
                bay_2_shape = point_normal
            else:
                incorrect_shapes.append(point_normal)
        if bay_1_shape == None:
            if len(self.incorrect_shapes) >= 1:
                print_bad("First bay not found, going in random bay")
                yield self.dock_in_bay_vision_only(incorrect_shapes[0])
                del incorrect_shapes[0]
            else:
                print_bad("First bay not found and no other bays found, moving on to second")
        else:
            print_good("Docking in first desired bay")
            yield self.dock_in_bay_vision_only(bay_1_shape)

        if bay_2_shape == None:
            if len(self.incorrect_shapes) >= 1:
                print_bad("Second bay not found, going in random bay")
                yield self.dock_in_bay_vision_only(incorrect_shapes[0])
                del incorrect_shapes[0]
            else:
                print_bad("Second bay not found and no other bays found")
        else:
            print_good("Docking in second desired bay")
            yield self.dock_in_bay_vision_only(bay_2_shape)

    @txros.util.cancellableInlineCallbacks
    def dock_in_bay_vision_only(self, (shapepoint, shapenormal)):
        move_front = self.navigator.move.set_position(shapepoint + shapenormal * self.LOOK_AT_DISTANCE).look_at(shapepoint)
        move_in    = self.navigator.move.set_position(shapepoint + shapenormal *  self.DOCK_DISTANCE).look_at(shapepoint)
        yield move_front.go()
        yield self.ogrid_clear.toggle()
        yield move_in.go()
        yield self.navigator.nh.sleep(self.DOCK_SLEEP_TIME)
        yield move_front.go()
        yield self.ogrid_clear.toggle()

    @txros.util.cancellableInlineCallbacks
    def find_and_dock_vision_only(self):
        yield self._init_vision_only()
        yield self.navigator.vision_proxies["get_shape_front"].start()
        yield self.get_target_bays()
        yield self.get_waypoint()
        yield self.circle_dock_vision_only()
        yield self.dock_vision_only()
        yield self.navigator.vision_proxies["get_shape_front"].stop()

@txros.util.cancellableInlineCallbacks
def setup_mission(navigator):
    bay_1_color = "GREEN"
    bay_1_shape = "TRIANGLE"
    bay_2_color = "RED"
    bay_2_shape = "CIRCLE"
    yield navigator.mission_params["dock_shape_2"].set(bay_2_shape)
    yield navigator.mission_params["dock_shape_1"].set(bay_1_shape)
    yield navigator.mission_params["dock_color_1"].set(bay_1_color)
    yield navigator.mission_params["dock_color_2"].set(bay_2_color)

@txros.util.cancellableInlineCallbacks
def main(navigator, **kwargs):
    yield setup_mission(navigator)
    print_good("STARTING MISSION")
    mission = IdentifyDockMission(navigator)
    yield mission.find_and_dock_vision_only()
    print_bad("ENDING MISSION")

