"""Handles the perception of the ScanTheCode Mission."""
from __future__ import division
from collections import deque
from scanthecode_model_tracker import ScanTheCodeModelTracker
from collections import Counter
from image_geometry import PinholeCameraModel
import numpy.linalg as npl
from rect_finder_clustering import RectangleFinderClustering
from color_finder import ColorFinder
from twisted.internet import defer
from cv_bridge import CvBridge, CvBridgeError
import cv2
import txros
import sensor_msgs.point_cloud2 as pc2
from sensor_msgs.msg import PointCloud2, Image
import sys
import numpy as np
from mil_misc_tools.text_effects import fprint
from mil_tools import rosmsg_to_numpy

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
        self.camera_model = PinholeCameraModel()
        self.my_tf = my_tf
        self.rect_finder = RectangleFinderClustering()
        self.color_finder = ColorFinder()

        self.count = 0
        self.depths = []

    @txros.util.cancellableInlineCallbacks
    def _init(self):
        self.vel_sub = yield self.nh.subscribe("/velodyne_points", PointCloud2)
        self.image_sub = yield self.nh.subscribe("/camera/front/right/image_rect_color", Image)
        defer.returnValue(self)

    def add_image(self, image):
        """Add an image to the image cache."""
        if len(self.image_cache) > 50:
            self.image_cache.popleft()
        self.image_cache.append(image)

    def update_info(self, info):
        """Update the camera calibration info."""
        self.last_cam_info = info
        self.camera_model.fromCameraInfo(info)

    def _get_top_left_point(self, points_3d):
        xmin = sys.maxsize
        zmin = -sys.maxsize
        ymin = sys.maxsize
        for i, point in enumerate(points_3d):
            if point[1] < ymin:
                xmin = point[0]
                zmin = point[2]
                ymin = point[1]

        buff = 1
        for i, point in enumerate(points_3d):
            if(point[1] < ymin + buff and point[1] > ymin - buff and point[0] < xmin):
                xmin = point[0]
                zmin = point[2]
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
        min_val, max_val = sys.maxsize, -sys.maxsize
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
        # xmin, ymin, zmin = self._get_top_left_point(points_3d)
        criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 10, 1.0)
        points_3d = np.float32(points_3d)
        ret, label, centers = cv2.kmeans(np.array(points_3d), 2, criteria, 10, 0)
        data = Counter(label.flatten())
        max_label = data.most_common(1)[0][0]
        c = centers[max_label]
        xmin, ymin, zmin = c[0], c[1], c[2]

        points_3d = [[xmin - .3, ymin - .3, zmin], [xmin + .3, ymin + .3, zmin],
                     [xmin - .3, ymin + .3, zmin], [xmin + .3, ymin - .3, zmin]]

        points_2d = map(lambda x: self.camera_model.project3dToPixel(x), points_3d)
        return points_2d

    def _get_bounding_rect(self, points_2d, img):
        xmin = np.inf
        xmax = -np.inf
        ymin = np.inf
        ymax = -np.inf
        h, w, r = img.shape
        for i, point in enumerate(points_2d):
            if(point[0] < xmin):
                xmin = point[0]
            if(point[0] > xmax):
                xmax = point[0]
            if(point[1] > ymax):
                ymax = point[1]
            if(point[1] < ymin):
                ymin = point[1]
        if xmin < 0:
            xmin = 1
        if ymin < 0:
            ymin = 1
        if xmax > w:
            xmax = w - 1
        if ymax > h:
            ymax = h - 1
        return xmin, ymin, xmax, ymax

    @txros.util.cancellableInlineCallbacks
    def get_stc_points(self, msg, stc_pos):
        trans = yield self.my_tf.get_transform("/front_right_cam_optical", "/velodyne", msg.header.stamp)
        trans1 = yield self.my_tf.get_transform("/front_right_cam_optical", "/enu", msg.header.stamp)
        stc_pos = rosmsg_to_numpy(stc_pos)
        stc_pos = np.append(stc_pos, 1)
        position = trans1.as_matrix().dot(stc_pos)
        if position[3] < 1E-15:
            raise ZeroDivisionError
        position[0] /= position[3]
        position[1] /= position[3]
        position[2] /= position[3]
        position = position[:3]

        stereo_points = []
        for point in pc2.read_points(msg, skip_nans=True):
            stereo_points.append(np.array([point[0], point[1], point[2], 1]))
        stereo_points = map(lambda x: trans.as_matrix().dot(x), stereo_points)
        points = []
        for p in stereo_points:
            if p[3] < 1E-15:
                raise ZeroDivisionError
            p[0] /= p[3]
            p[1] /= p[3]
            p[2] /= p[3]
            points.append(p[:3])

        points_keep = []
        for p in points:
            # print npl.norm(p - poition)
            if npl.norm(p - position) < 20:
                points_keep.append(p)
        points_keep = sorted(points_keep, key=lambda x: x[1])
        keep_num = int(.1 * len(points_keep))
        points_keep = points_keep[:keep_num]
        # self.pers_points.extend(points_keep)
        # max_num = 200
        # if len(self.pers_points) > max_num:
        #     self.pers_points = self.pers_points[len(self.pers_points) - max_num:len(self.pers_points)]

        defer.returnValue(points_keep)

    @txros.util.cancellableInlineCallbacks
    def search(self, scan_the_code):
        """Search for the colors in the scan the code object."""
        pntcloud = yield self.vel_sub.get_next_message()
        image_ros = yield self.image_sub.get_next_message()
        try:
            image = self.bridge.imgmsg_to_cv2(image_ros, "bgr8")
        except CvBridgeError:
            print "Trouble converting image"
            defer.returnValue((False, None))

        points_3d = yield self.get_stc_points(pntcloud, scan_the_code.position)

        image_clone = image.copy()

        # points_3d = yield self._get_3d_points_stereo(scan_the_code.points, image_ros.header.stamp)
        # points_2d = map(lambda x: self.camera_model.project3dToPixel(x), points_3d)
        points_2d = map(lambda x: self.camera_model.project3dToPixel(x), points_3d)
        for p in points_2d:
            po = (int(round(p[0])), int(round(p[1])))
            cv2.circle(image_clone, po, 2, (0, 255, 0), -1)

        points_2d = self._get_2d_points_stc(points_3d)
        xmin, ymin, xmax, ymax = self._get_bounding_rect(points_2d, image)
        xmin, ymin, xmax, ymax = int(xmin), int(ymin), int(xmax), int(ymax)

        cv2.rectangle(image_clone, (xmin, ymin), (xmax, ymax), (0, 255, 0), 2)
        self.debug.add_image(image_clone, "bounding_box", topic="bounding_box")

        # defer.returnValue((False, None))

        print xmin, ymin, xmax, ymax
        roi = image[ymin:ymax, xmin:xmax]
        succ, color_vec = self.rect_finder.get_rectangle(roi, self.debug)
        if not succ:
            defer.returnValue((False, None))

        self.mission_complete, colors = self.color_finder.check_for_colors(image, color_vec, self.debug)
        if self.mission_complete:
            print "MISSION COMPLETE"
            defer.returnValue((True, colors))
        defer.returnValue((False, None))

    @txros.util.cancellableInlineCallbacks
    def correct_pose(self, scan_the_code):
        """Check to see if we are looking at the corner of scan the code."""
        self.count += 1
        pntcloud = yield self.vel_sub.get_next_message()
        points_3d = yield self.get_stc_points(pntcloud, scan_the_code.position)
        xmin, ymin, zmin = self._get_top_left_point(points_3d)
        points_oi = self._get_points_in_range('y', ymin - .1, ymin + .2, points_3d)
        if len(points_oi) == 0:
            defer.returnValue(False)

        image_ros = yield self.image_sub.get_next_message()
        image_ros = self.bridge.imgmsg_to_cv2(image_ros, "bgr8").copy()

        depth = self._get_depth('z', points_oi)
        fprint("DEPTH: {}".format(depth), msg_color="green")
        self.depths.append(depth)
        # %%%%%%%%%%%%%%%%%%%%%%%%DEBUG
        cv2.putText(image_ros, str(depth), (10, 30), 1, 2, (0, 0, 255))
        self.debug.add_image(image_ros, "in_range", topic="in_range")
        # %%%%%%%%%%%%%%%%%%%%%%%%DEBUG
        if depth > .10:
            defer.returnValue(False)
        defer.returnValue(True)
