from __future__ import division
import tf
import rospy
import numpy as np
from twisted.internet import defer
from txros import util
from sensor_msgs.msg import RegionOfInterest
from sensor_msgs.msg import CameraInfo
from image_geometry import PinholeCameraModel
from mil_misc_tools import text_effects
from geometry_msgs.msg import Vector3, Point
from mil_misc_tools import FprintFactory
from std_msgs.msg import String
from visualization_msgs.msg import *

MISSION = 'Path Marker Challenge'


class PathFollower(object):
    '''
    Mission code to align to path markers and go on the Path.
    '''

    X_PATTERN_RADIUS = rospy.get_param('~pattern_x', .5)
    Y_PATTERN_RADIUS = rospy.get_param('pattern_y', .5)
    PURSUIT_RANGE = rospy.get_param('~pursuit_range', 1)
    STARTING_DEPTH = rospy.get_param('~start_depth', -1)
    START_GATE_POS = rospy.get_param('~start_gate', [0, 0, 1])
    MOVE_STEP = rospy.get_param('~self.move_step', .2)
    BLIND = rospy.get_param('~blind', True)
    SCALE = rospy.get_param('~init_scale', 1)
    SLEEP_TIME = rospy.get_param('~sleep_time', 1)

    def __init__(self, sub):
        self.sub = sub
        self.print_info = FprintFactory(title=MISSION).fprint
        self.print_bad = FprintFactory(title=MISSION, msg_color="red").fprint
        self.print_good = FprintFactory(
            title=MISSION, msg_color="green").fprint
        self.pattern_done = False
        self.done = False
        self.generate_pattern()
        # self.get_path_marker = self.sub.nh.subscribe(
        # '/vision/path_roi', RegionOfInterest)
        self.get_direction = self.sub.nh.subscribe(
            '/vision/path_direction', String)
        self.get_orange = self.sub.nh.subscribe('/vision/path_orange', String)
        self.t = None
        self.reset = True
        self.starting_pos = sub.tx_pose()

    def generate_pattern(self):
        '''
        Generates a box search pattern. Logically if we move backward,
        if we are aligned, we should see it right away. If not the pattern continues.
        First we move left, then down, then right twice, then up, then back to start.
        If we still haven't seen it, we scale up the grid, all in meters.
        '''
        x = self.X_PATTERN_RADIUS
        y = self.Y_PATTERN_RADIUS
        z = self.STARTING_DEPTH
        s = self.SCALE
        self.moves = [[0, s * y, 0], [s * x, 0, 0], [0, s * -2 * y, 0],
                      [-2 * x * s, 0, 0], [0, s * y, 0], [s * x, 0, 0]]
        self.move_index = 0

    @util.cancellableInlineCallbacks
    def search(self):
        '''
        Search for orange pixels on our screen. If we locate orange pixels,
        move towards them a certain distance and see if there is a path marker.
        '''
        self.print_info('Starting Search Pattern')

        while True:
            yield self.sub.nh.sleep(self.SLEEP_TIME)
            # path_marker = yield self.get_path_marker.get_last_message()
            get_dir = yield self.get_direction.get_last_message()
            orange_pixel = yield self.get_orange.get_last_message()
            # self.print_info(('Orange Pixel Direction, ', orange_pixel))
            # self.print_info(('Direction, ', get_dir))
            # self.print_info(('Path Marker, ', path_marker))
            if orange_pixel is not None:
                if get_dir is not None and orange_pixel.data == 'center':
                    self.print_good('Marker Acquired.')
                    self.t = 'Acquired'
                    self.reset = True
                    self.pattern_done = True
                    if get_dir.data == 'left':
                        self.print_good('Marker Found, Looking Left')
                        # yield self.sub.move.look_at_without_pitching(-1 *
                        # self.START_GATE_POS).go(blind=self.BLIND)
                        yield self.sub.move.yaw_left(45).go(blind=self.BLIND)
                        self.done = True
                        break
                    elif get_dir.data == 'right':
                        self.print_good('Marker Found, Looking Right')
                        # yield self.sub.move.look_at_without_pitching(-1 *
                        # self.START_GATE_POS).go(blind=self.BLIND)
                        yield self.sub.move.yaw_right(45).go(blind=self.BLIND)
                        self.done = True
                        break
                elif orange_pixel.data == 'top':
                    self.pattern_done = True
                    # move left     f/b  l/r u/d
                    move = np.array([0, self.MOVE_STEP, 0])
                    yield self.sub.move.relative(move).go(blind=self.BLIND)
                elif orange_pixel.data == 'top_left':
                    self.pattern_done = True
                    # move back left
                    move = np.array([-self.MOVE_STEP, self.MOVE_STEP, 0])
                    yield self.sub.move.relative(move).go(blind=self.BLIND)
                    yield self.sub.move.relative(move).go(blind=self.BLIND)
                elif orange_pixel.data == 'top_right':
                    # move forward left
                    self.pattern_done = True
                    move = np.array([self.MOVE_STEP, self.MOVE_STEP, 0])
                    yield self.sub.move.relative(move).go(blind=self.BLIND)
                    yield self.sub.move.relative(move).go(blind=self.BLIND)
                elif orange_pixel.data == 'bot':
                    # move right
                    self.pattern_done = True
                    move = np.array([0, -self.MOVE_STEP, 0])
                    yield self.sub.move.relative(move).go(blind=self.BLIND)
                elif orange_pixel.data == 'bot_left':
                    # move backward left
                    self.pattern_done = True
                    move = np.array([-self.MOVE_STEP, -self.MOVE_STEP, 0])
                    yield self.sub.move.relative(move).go(blind=self.BLIND)
                    yield self.sub.move.relative(move).go(blind=self.BLIND)
                elif orange_pixel.data == 'bot_right':
                    # move forward right
                    # print('moving forward, right')
                    self.pattern_done = True
                    move = np.array([self.MOVE_STEP, -self.MOVE_STEP, 0])
                    yield self.sub.move.relative(move).go(blind=self.BLIND)
                    yield self.sub.move.relative(move).go(blind=self.BLIND)
                elif orange_pixel.data == 'left':
                    # move backward
                    self.pattern.cancel()
                    move = np.array([-self.MOVE_STEP, 0, 0])
                    yield self.sub.move.relative(move).go(blind=self.BLIND)
                elif orange_pixel.data == 'right':
                    # move forward
                    self.pattern_done = True
                    move = np.array([self.MOVE_STEP, 0, 0])
                    yield self.sub.move.relative(move).go(blind=self.BLIND)
                else:
                    self.pattern_done = False

    @util.cancellableInlineCallbacks
    def pattern(self):
        def err():
            self.print_info('Search pattern canceled')

        self.pattern_done = False
        for i, move in enumerate(self.moves[self.move_index:]):
            if self.pattern_done:
                break
            yield self.sub.nh.sleep(self.SLEEP_TIME)
            info = 'Performing move ', move
            self.print_info(info)
            move = self.sub.move.relative(np.array(move)).go(blind=self.BLIND)
            move.addErrback(err)
            yield move
            self.move_index = i + 1
        self.print_bad('Pattern finished.')
        self.reset = not self.reset
        self.pattern_done = True

    @util.cancellableInlineCallbacks
    def run(self):
        # start_time = yield self.sub.nh.get_time()  # Store time mission
        # starts for timeout

        self.print_info("Enabling Perception")
        self.sub.vision_proxies.the_path.start()

        pattern = self.pattern()
        self.do_search = True
        search = self.search()
        while not self.done:
            if self.pattern_done is True:
                pattern.cancel()
            elif self.pattern_done is True and self.reset == False:
                self.SCALE += 1
                self.generate_pattern()
                pattern = self.pattern()
                self.reset = True
            yield self.sub.nh.sleep(0.1)
        search.cancel()
        pattern.cancel()
        self.sub.vision_proxies.the_path.stop()
        self.print_good('Done!')


@util.cancellableInlineCallbacks
def run(sub):
    mission = PathFollower(sub)
    yield mission.run()
