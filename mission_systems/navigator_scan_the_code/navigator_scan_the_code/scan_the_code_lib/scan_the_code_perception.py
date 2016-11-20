"""Handles the perception of the ScanTheCode Mission."""
from collections import deque
from scanthecode_model_tracker import ScanTheCodeModelTracker
from image_geometry import PinholeCameraModel
from rect_finder import RectangleFinder
from twisted.internet import defer
from cv_bridge import CvBridge, CvBridgeError
import cv2
import txros
import matplotlib.pyplot as plt
import sys
import numpy as np
import genpy
from navigator_tools import fprint

___author___ = "Tess Bianchi"


class ScanTheCodePerception(object):
    """Class that handles ScanTheCodePerception."""

    def __init__(self, my_tf, debug, nh):
        """Initialize ScanTheCodePerception class."""
        self.image_cache = deque()
        self.bridge = CvBridge()
        self.nh = nh
        self.last_cam_info = None
        self.debug = debug
        self.pers_points = []
        self.model_tracker = ScanTheCodeModelTracker()
        self.pinhole_cam = PinholeCameraModel()
        self.my_tf = my_tf

        # %%%%%%%%%%%%debug%%%%%%%%%%%%%%%%
        self.count = 0
        self.depths = []

    def add_image(self, image):
        """Add an image to the image cache."""
        if len(self.image_cache) > 20:
            self.image_cache.popleft()
        self.image_cache.append(image)

    def update_info(self, info):
        """Update the camera calibration info."""
        self.last_cam_info = info

    def _convert_3d_2d(self, point):
        K = self.last_cam_info.K
        K = np.array(K).reshape(3, 3)
        pl = point
        pl = K.dot(pl)
        if pl[2] == 0:
            return (0, 0)
        pl[0] /= pl[2]
        pl[1] /= pl[2]
        if pl[0] < 0:
            pl[0] = 0
        if pl[1] < 0:
            pl[1] = 0
        return(int(pl[0]), int(pl[1]))

    @txros.util.cancellableInlineCallbacks
    def _get_3d_points_stereo(self, points_3d_enu, time):
        self.pers_points.extend(points_3d_enu)
        max_num = 1000
        if len(self.pers_points) > max_num:
            self.pers_points = self.pers_points[len(self.pers_points) - max_num:len(self.pers_points)]

        points_3d = []
        try:
            trans = yield self.my_tf.get_transform("/stereo_right_cam", "/enu", time)
        except Exception as exp:
            print exp
            defer.returnValue(points_3d)

        transformation = trans.as_matrix()
        for point in self.pers_points:
            p = [point.x, point.y, point.z, 1]
            t_p = transformation.dot(p)
            if t_p[3] < 1E-15:
                raise ZeroDivisionError
            t_p[0] /= t_p[3]
            t_p[1] /= t_p[3]
            t_p[2] /= t_p[3]
            points_3d.append(t_p[0:3])
        defer.returnValue(points_3d)

    def _get_top_left_point(self, points_3d):
        xmin = sys.maxint
        zmin = -sys.maxint
        ymin = sys.maxint
        for i, point in enumerate(points_3d):
            if point[1] < ymin:
                xmin = point[0]
                zmin = point[2]
                ymin = point[1]

        buff = .07
        for i, point in enumerate(points_3d):
            if(point[1] < ymin + buff and point[1] > ymin - buff and point[0] < xmin):
                xmin = point[0]
                zmin = point[2]
                ymin = point[1]
        return xmin, ymin, zmin

    def _get_points_in_range(self, axis, lower, upper, points):
        in_range = []
        idx = 0
        if axis == 'y':
            idx = 1
        if axis == 'z':
            idx = 2
        for p in points:
            if p[idx] > lower and p[idx] < upper:
                in_range.append(p)

        return in_range

    def _get_depth(self, axis, points_3d):
        idx = 0
        if axis == 'y':
            idx = 1
        if axis == 'z':
            idx = 2
        min_val, max_val = sys.maxint, -sys.maxint
        points_3d = np.array(points_3d)
        my_points = points_3d[:, idx]
        mean = np.mean(my_points)
        std = 1.5 * np.std(my_points)
        for p in my_points:
            if abs(p - mean) > std:
                # print p, mean
                continue
            if p < min_val:
                min_val = p
            if p > max_val:
                max_val = p

        return max_val - min_val

    def _get_2d_points_stc(self, points_3d):
        xmin, ymin, zmin = self._get_top_left_point(points_3d)
        ymin -= .15
        xmin -= .05

        points_3d = [[xmin, ymin, zmin], [xmin + .8, ymin + .75, zmin], [xmin, ymin + .75, zmin], [xmin + .8, ymin, zmin]]

        points_2d = []
        for i in range(0, len(points_3d)):
            point = points_3d[i]
            points_2d.append(self._convert_3d_2d(point))
        return points_2d

    def _get_rectangle(self, img, bounding_rect):
        xmin, ymin, xmax, ymax = bounding_rect
        roi = img[ymin:ymax, xmin:xmax]
        r = RectangleFinder()
        return r.get_rectangle(roi, self.debug)

    def _get_bounding_rect(self, points_2d):
        xmin = 1000
        xmax = 0
        ymin = 1000
        ymax = 0
        for i, point in enumerate(points_2d):
            if(point[0] < xmin):
                xmin = point[0]
            if(point[0] > xmax):
                xmax = point[0]
            if(point[1] > ymax):
                ymax = point[1]
            if(point[1] < ymin):
                ymin = point[1]
        return xmin, ymin, xmax, ymax

    def _get_closest_image(self, time):
        min_img = None
        min_diff = genpy.Duration(sys.maxint)
        for img in self.image_cache:
            diff = abs(time - img.header.stamp)
            if diff < min_diff:
                min_diff = diff
                min_img = img
        return min_img

    @txros.util.cancellableInlineCallbacks
    def search(self, scan_the_code):
        """Search for the colors in the scan the code object."""
        if len(self.image_cache) == 0:
            defer.returnValue((False, None))
        image_ros = self._get_closest_image(scan_the_code.header.stamp)
        try:
            image = self.bridge.imgmsg_to_cv2(image_ros, "bgr8")
        except CvBridgeError:
            print "Trouble converting image"
            defer.returnValue((False, None))

        image_clone = image.copy()

        points_3d = yield self._get_3d_points_stereo(scan_the_code.points, image_ros.header.stamp)
        for i in points_3d:
            p = self._convert_3d_2d(i)
            po = (int(p[0]), int(p[1]))
            cv2.circle(image_clone, po, 2, (0, 255, 0), -1)
        points_2d = self._get_2d_points_stc(points_3d)
        self.debug.add_image(image_clone, "bounding_box", topic="bounding_box")

        xmin, ymin, xmax, ymax = self._get_bounding_rect(points_2d)
        xmin, ymin, xmax, ymax = int(xmin), int(ymin), int(xmax), int(ymax)

        cv2.rectangle(image_clone, (xmin, ymin), (xmax, ymax), (255, 0, 0), 2)

        h, w, r = image.shape

        if ymin == ymax or xmin == xmax or ymin < 0 or ymax > h or xmin < 0 or xmax > w:
            defer.returnValue((False, None))

        cv2.rectangle(image_clone, (xmin, ymin), (xmax, ymax), (255, 0, 0), 2)
        self.debug.add_image(image_clone, "bounding_box", topic="bounding_box")

        succ, rect = self._get_rectangle(image, (xmin, ymin, xmax, ymax))

        if not succ:
            defer.returnValue((False, None))

        pnts = []
        for p in rect:
            p1 = p[0] + xmin
            p2 = p[1] + ymin
            pnts.append((int(p1), int(p2)))

        self.mission_complete = self.model_tracker.update_model(image, pnts, self.debug)
        if(self.mission_complete):
            print "MISSION COMPLETE"
            defer.returnValue((True, self.model_tracker.colors))
        else:
            defer.returnValue((False, None))

        yield True

    @txros.util.cancellableInlineCallbacks
    def correct_pose(self, scan_the_code):
        """Check to see if we are looking at the corner of scan the code."""
        self.count += 1
        # %%%%%%%%%%%%%%%%%%%%%%%%DEBUG
        # if self.count == 100:
        #     xs = np.arange(0, len(self.depths))
        #     ys = self.depths
        #     plt.plot(xs, ys)
        #     plt.show()
        # %%%%%%%%%%%%%%%%%%%%%%%%DEBUG

        points_3d = yield self._get_3d_points_stereo(scan_the_code.points, self.nh.get_time())
        xmin, ymin, zmin = self._get_top_left_point(points_3d)
        points_oi = self._get_points_in_range('y', ymin - .1, ymin + .2, points_3d)
        if len(points_oi) == 0:
            defer.returnValue(False)

        # %%%%%%%%%%%%%%%%%%%%%%%%DEBUG
        image_ros = self._get_closest_image(scan_the_code.header.stamp)
        count = 0
        while image_ros is None:
            yield self.nh.sleep(.5)
            image_ros = self._get_closest_image(scan_the_code.header.stamp)
            yield self.nh.sleep(.3)
            count += 1
            if count == 20:
                defer.returnValue(False)
        # %%%%%%%%%%%%%%%%%%%%%%%%DEBUG

        image_ros = self.bridge.imgmsg_to_cv2(image_ros, "bgr8").copy()

        depth = self._get_depth('z', points_oi)
        fprint("DEPTH: {}".format(depth), msg_color="green")
        self.depths.append(depth)
        if depth > .15:
            # %%%%%%%%%%%%%%%%%%%%%%%%DEBUG
            cv2.putText(image_ros, str(depth), (10, 30), 1, 2, (0, 0, 255))
            self.debug.add_image(image_ros, "in_range", topic="in_range")
            # %%%%%%%%%%%%%%%%%%%%%%%%DEBUG
            defer.returnValue(False)

        # %%%%%%%%%%%%%%%%%%%%%%%%DEBUG
        cv2.putText(image_ros, str(depth), (10, 30), 1, 2, (0, 255, 0))
        self.debug.add_image(image_ros, "in_range1", topic="in_range")
        # %%%%%%%%%%%%%%%%%%%%%%%%DEBUG
        defer.returnValue(True)
