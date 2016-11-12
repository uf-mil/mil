#!/usr/bin/env python
"""Runs all of the mission components of Scan The Code."""
import txros
import genpy
from twisted.internet import defer
import sys
from sensor_msgs.msg import Image, CameraInfo
from scan_the_code_lib import ScanTheCodeAction, ScanTheCodePerception, Debug
from navigator_msgs.srv import ObjectDBQuery, ObjectDBQueryRequest
from navigator_tools import MissingPerceptionObject
___author___ = "Tess Bianchi"


class ScanTheCodeMission:
    """Class that contains all the functionality for Scan The Code."""

    def __init__(self, nh):
        """Initialize ScanTheCodeMission class."""
        self.nh = nh
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
        self.perception = ScanTheCodePerception(my_tf, self.debug, self.nh)
        self.database = yield self.nh.get_service_client("/database/requests", ObjectDBQuery)
        self.image_sub = yield self.nh.subscribe("/stereo/left/image_rect_color", Image, self._image_cb)
        self.cam_info_sub = yield self.nh.subscribe("/stereo/left/camera_info", CameraInfo, self._info_cb)

    def _image_cb(self, image):
        self.perception.add_image(image)

    def _info_cb(self, info):
        self.perception.update_info(info)

    @txros.util.cancellableInlineCallbacks
    def _get_scan_the_code(self):
        req = ObjectDBQueryRequest()
        req.name = 'scan_the_code'
        scan_the_code = yield self.database(req)
        if len(scan_the_code.objects) == 0:
            print "Missing scan the code"
            raise MissingPerceptionObject('scan_the_code')
        defer.returnValue(scan_the_code.objects[0])

    @txros.util.cancellableInlineCallbacks
    def find_colors(self, timeout=sys.maxint):
        """Find the colors of scan the code."""
        length = genpy.Duration(timeout)
        start = self.nh.get_time()
        while start - self.nh.get_time() < length:
            scan_the_code = yield self._get_scan_the_code()

            try:
                success, colors = yield self.perception.search(scan_the_code)
                if success:
                    defer.returnValue(colors)
            except Exception as e:
                print e
                yield self.nh.sleep(.1)
                continue

            yield self.nh.sleep(.3)
        defer.returnValue(None)

    @txros.util.cancellableInlineCallbacks
    def initial_position(self, timeout=sys.maxint):
        """Get the initial position of scan the code."""
        length = genpy.Duration(timeout)
        start = self.nh.get_time()
        while start - self.nh.get_time() < length:
            scan_the_code = yield self._get_scan_the_code()
            defer.returnValue(self.action.initial_position(scan_the_code))

    @txros.util.cancellableInlineCallbacks
    def correct_pose(self, pose, timeout=sys.maxint):
        """Check to see if the boat pose needs to be corrected to get an optimal viewing angle."""
        length = genpy.Duration(timeout)
        start = self.nh.get_time()
        while start - self.nh.get_time() < length:
            scan_the_code = yield self._get_scan_the_code()
            print "gottittt"
            correct_pose = yield self.perception.correct_pose(scan_the_code)
            if correct_pose:
                self.stc_correct = True
                defer.returnValue(True)

            yield self.nh.sleep(.1)
