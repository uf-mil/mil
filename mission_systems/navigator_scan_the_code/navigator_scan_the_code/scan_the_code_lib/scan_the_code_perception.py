from collections import deque
from model_tracker import ModelTracker
from image_geometry import PinholeCameraModel
from rect_finder import RectangleFinder
from twisted.internet import defer
from cv_bridge import CvBridge, CvBridgeError
import cv2
import txros
import sys
import numpy as np
import genpy
import matplotlib.pyplot as plt


class ScanTheCodePerception(object):

    def __init__(self, my_tf, debug, nh):
        self.image_cache = deque()
        self.bridge = CvBridge()
        self.nh = nh
        self.last_cam_info = None
        self.debug = debug
        self.pers_points = []
        self.model_tracker = ModelTracker()
        self.pinhole_cam = PinholeCameraModel()
        self.my_tf = my_tf

        # %%%%%%%%%%%%debug%%%%%%%%%%%%%%%%
        self.count = 0
        self.depths = []

    def add_image(self, image):
        if len(self.image_cache) > 20:
            self.image_cache.popleft()
        self.image_cache.append(image)

    def update_info(self, info):
        self.last_cam_info = info

    def convert_3d_2d(self, point):
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
    def get_3d_points_stereo(self, points_3d_enu, time):
        self.pers_points.extend(points_3d_enu)
        max_num = 1000
        if len(self.pers_points) > max_num:
            self.pers_points = self.pers_points[len(self.pers_points) - max_num:len(self.pers_points)]

        points_3d = []
        try:
            trans = yield self.my_tf.get_transform("/stereo_left_cam", "/enu", time)
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

    def get_top_left_point(self, points_3d):
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

    def get_points_in_range(self, axis, lower, upper, points):
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

    def get_depth(self, axis, points_3d):
        idx = 0
        if axis == 'y':
            idx = 1
        if axis == 'z':
            idx = 2
        min_val, max_val = sys.maxint, -sys.maxint
        for p in points_3d:
            if p[idx] < min_val:
                min_val = p[idx]
            if p[idx] > max_val:
                max_val = p[idx]

        return max_val - min_val

    def get_2d_points_stc(self, points_3d):
        xmin, ymin, zmin = self.get_top_left_point(points_3d)
        ymin -= .15
        xmin -= .05

        points_3d = [[xmin, ymin, zmin], [xmin + .8, ymin + .75, zmin], [xmin, ymin + .75, zmin], [xmin + .8, ymin, zmin]]

        points_2d = []
        for i in range(0, len(points_3d)):
            point = points_3d[i]
            points_2d.append(self.convert_3d_2d(point))
        return points_2d

    def get_rectangle(self, img, bounding_rect):
        xmin, ymin, xmax, ymax = bounding_rect
        roi = img[ymin:ymax, xmin:xmax]
        r = RectangleFinder()
        return r.get_rectangle(roi, self.debug)

    def get_bounding_rect(self, points_2d):
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

    def get_closest_image(self, time):
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

        if len(self.image_cache) == 0:
            defer.returnValue((False, None))
        image_ros = self.get_closest_image(scan_the_code.header.stamp)
        try:
            image = self.bridge.imgmsg_to_cv2(image_ros, "bgr8")
        except CvBridgeError:
            print "Trouble converting image"
            defer.returnValue((False, None))

        image_clone = image.copy()

        points_3d = yield self.get_3d_points_stereo(scan_the_code.points, image_ros.header.stamp)
        for i in points_3d:
            p = self.convert_3d_2d(i)
            po = (int(p[0]), int(p[1]))
            cv2.circle(image_clone, po, 2, (0, 255, 0), -1)
        points_2d = self.get_2d_points_stc(points_3d)
        self.debug.add_image(image_clone, "bounding_box", topic="bounding_box")

        xmin, ymin, xmax, ymax = self.get_bounding_rect(points_2d)
        xmin, ymin, xmax, ymax = int(xmin), int(ymin), int(xmax), int(ymax)

        cv2.rectangle(image_clone, (xmin, ymin), (xmax, ymax), (255, 0, 0), 2)

        if ymin == ymax or xmin == xmax:
            defer.returnValue((False, None))

        cv2.rectangle(image_clone, (xmin, ymin), (xmax, ymax), (255, 0, 0), 2)
        self.debug.add_image(image_clone, "bounding_box", topic="bounding_box")

        succ, rect = self.get_rectangle(image, (xmin, ymin, xmax, ymax))

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
        self.count += 1
        # if self.count == 100:
        #     xs = np.arange(0, len(self.depths))
        #     ys = self.depths
        #     plt.plot(xs, ys)
        #     plt.show()

        points_3d = yield self.get_3d_points_stereo(scan_the_code.points, self.nh.get_time())
        xmin, ymin, zmin = self.get_top_left_point(points_3d)
        points_oi = self.get_points_in_range('y', ymin - .1, ymin + .2, points_3d)
        if len(points_oi) == 0:
            defer.returnValue(False)
        # %%%%%%%%%%%%%%%%%%%%%%%%DEBUG

        image_ros = self.get_closest_image(scan_the_code.header.stamp)
        while image_ros is None:
            image_ros = self.get_closest_image(scan_the_code.header.stamp)

        image_ros = self.bridge.imgmsg_to_cv2(image_ros, "bgr8").copy()

        depth = self.get_depth('z', points_oi)
        self.depths.append(depth)
        if depth > .3:
            cv2.putText(image_ros, str(depth), (10, 30), 1, 2, (0, 0, 255))
            self.debug.add_image(image_ros, "in_range", topic="in_range")
            defer.returnValue(False)
        else:
            cv2.putText(image_ros, str(depth), (10, 30), 1, 2, (0, 255, 0))
            self.debug.add_image(image_ros, "in_range1", topic="in_range")
            defer.returnValue(True)
