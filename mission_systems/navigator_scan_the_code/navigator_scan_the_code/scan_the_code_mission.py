#!/usr/bin/env python
"""Runs all of the mission components of Scan The Code."""
import txros
import genpy
from twisted.internet import defer
import sys
from sensor_msgs.msg import CameraInfo
from scan_the_code_lib import ScanTheCodeAction, ScanTheCodePerception, Debug
from navigator_msgs.srv import ObjectDBQuery
from navigator_tools import DBHelper
from mil_tools import fprint
___author___ = "Tess Bianchi"


class ScanTheCodeMission:
    """Class that contains all the functionality for Scan The Code."""

    def __init__(self, navigator):
        """Initialize ScanTheCodeMission class."""
        self.nh = navigator.nh
        self.navigator = navigator
        self.action = ScanTheCodeAction()
        self.mission_complete = False
        self.colors = []
        self.scan_the_code = None

        self.stc_correct = False

    @txros.util.cancellableInlineCallbacks
    def init_(self, tl):
        """Initialize the txros elements of ScanTheCodeMission class."""
        my_tf = tl
        self.debug = Debug(self.nh, wait=False)
        self.perception = yield ScanTheCodePerception(my_tf, self.debug, self.nh)._init()
        self.database = yield self.nh.get_service_client("/database/requests", ObjectDBQuery)
        # self.image_sub = yield self.nh.subscribe("/camera/front/right/image_rect_color", Image, self._image_cb)
        self.cam_info_sub = yield self.nh.subscribe("/camera/front/right/camera_info", CameraInfo, self._info_cb)
        self.helper = yield DBHelper(self.nh).init_(self.navigator)

    def _info_cb(self, info):
        self.perception.update_info(info)

    @txros.util.cancellableInlineCallbacks
    def _get_scan_the_code(self):
        v = False
        if self.scan_the_code is None:
            ans = yield self.helper.get_object("scan_the_code", volume_only=v)
        else:
            try:
                ans = yield self.helper.get_object_by_id(self.scan_the_code.id)
            except Exception:
                print "PROBLEM"
                ans = yield self.helper.get_object("scan_the_code", volume_only=v)

        fprint("GOT SCAN THE CODE WITH ID {}".format(ans.id), msg_color="blue")
        defer.returnValue(ans)

    @txros.util.cancellableInlineCallbacks
    def find_colors(self, timeout=sys.maxsize):
        """Find the colors of scan the code."""
        length = genpy.Duration(timeout)
        start = self.nh.get_time()
        while start - self.nh.get_time() < length:
            try:
                scan_the_code = yield self._get_scan_the_code()
            except Exception:
                print "Could not get scan the code..."
                yield self.nh.sleep(.01)
                continue

            try:
                success, colors = yield self.perception.search(scan_the_code)
                if success:
                    defer.returnValue(colors)
            except Exception as e:
                print e
                yield self.nh.sleep(.1)
                continue

            # yield self.nh.sleep(.03)
        defer.returnValue(None)

    @txros.util.cancellableInlineCallbacks
    def initial_position(self, timeout=sys.maxsize):
        """Get the initial position of scan the code."""
        length = genpy.Duration(timeout)
        start = self.nh.get_time()
        while start - self.nh.get_time() < length:
            try:
                scan_the_code = yield self._get_scan_the_code()
            except Exception as exc:
                print exc
                print "Could not get scan the code..."
                yield self.nh.sleep(.1)
                continue

            defer.returnValue(self.action.initial_position(scan_the_code))

    @txros.util.cancellableInlineCallbacks
    def correct_pose(self, pose, timeout=sys.maxsize):
        """Check to see if the boat pose needs to be corrected to get an optimal viewing angle."""
        length = genpy.Duration(timeout)
        start = self.nh.get_time()
        while start - self.nh.get_time() < length:
            try:
                scan_the_code = yield self._get_scan_the_code()
            except Exception as exc:
                print exc
                print "Could not get scan the code..."
                yield self.nh.sleep(.1)
                continue

            correct_pose = yield self.perception.correct_pose(scan_the_code)
            if correct_pose:
                self.stc_correct = True
                defer.returnValue(True)
                break

            yield self.nh.sleep(.1)
