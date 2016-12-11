import txros
from twisted.internet import defer
from txros import util, tf
import navigator_tools as nt
from navigator_tools import Debug
import sys
from collections import deque
from cv_bridge import CvBridge
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Image, CameraInfo
import genpy
import cv2
import navigator_tools as nt
from navigator_msgs.srv import ObjectDBQuery, ObjectDBQueryRequest
from image_geometry import PinholeCameraModel
import numpy as np
___author___ = "Tess Bianchi"


class LidarToImage(object):

    def __init__(self, nh, training=False, classes=None, dist=50):
        self.MAX_SIZE = 74
        self.IMAGE_SIZE = 100
        self.max_dist = dist
        self.bridge = CvBridge()
        self.nh = nh
        self.id_to_perist = {}
        self.image_cache = deque()
        self.pose = None
        self.training = training
        self.image = None
        self.classes = classes
        self.cam_info = None
        self.busy = False
        self.c = 0

        self.debug = Debug(nh)

    @util.cancellableInlineCallbacks
    def init_(self, cam="r"):
        image_sub = "/stereo/right/image_rect_color"
        self.tf_frame = "/stereo_right_cam"
        cam_info = "/stereo/right/camera_info"
        if cam == 'l':
            image_sub = "/stereo/left/image_rect_color"
            self.tf_frame = "/stereo_left_cam"
            cam_info = "/stereo/left/camera_info"

        if cam == 'rr':
            image_sub = "/right/right/image_rect_color"
            self.tf_frame = "/right_right_cam"
            cam_info = "/right/right/camera_info"

        yield self.nh.subscribe(image_sub, Image, self._img_cb)
        self._database = yield self.nh.get_service_client('/database/requests', ObjectDBQuery)
        self._odom_sub = yield self.nh.subscribe('/odom', Odometry,
                                                 lambda odom: setattr(self, 'pose', nt.odometry_to_numpy(odom)[0]))
        self.cam_info_sub = yield self.nh.subscribe(cam_info, CameraInfo, self._info_cb)
        self.tf_listener = tf.TransformListener(self.nh)
        defer.returnValue(self)

    def _info_cb(self, info):
        self.cam_info = info

    @util.cancellableInlineCallbacks
    def get_all_object_rois(self):
        req = ObjectDBQueryRequest()
        req.name = 'all'
        obj = yield self._database(req)
        if obj is None or not obj.found:
            defer.returnValue((None, None))
        rois = []
        ros_img = yield self._get_closest_image(obj.objects[0].header.stamp)
        if ros_img is None:
            defer.returnValue((None, None))
        img = self.bridge.imgmsg_to_cv2(ros_img, "mono8")
        for o in obj.objects:
            if o.id not in self.id_to_perist:
                self.id_to_perist[o.id] = []
            ppoints = self.id_to_perist[o.id]
            ppoints.extend(o.points)
            if len(ppoints) > 500:
                ppoints = ppoints[:500]
            if self.training and o.name not in self.classes:
                continue
            position = yield self._get_position()
            o_pos = nt.rosmsg_to_numpy(o.position)
            diff = np.linalg.norm(position - o_pos)
            if diff > self.max_dist:
                continue
            points, bbox = yield self._get_bounding_box_2d(ppoints, obj.objects[0].header.stamp)
            if bbox is None:
                continue

            xmin, ymin, xmax, ymax = bbox

            h, w, r = img.shape
            if xmin < 0 or xmax < 0 or xmin > w or xmax > w or xmax - xmin == 0 or ymax - ymin == 0:
                continue
            if ymin < 0:
                ymin = 0
            print "bbox", bbox
            roi = img[ymin:ymax, xmin:xmax]
            print "preshape", roi.shape
            roi = self._resize_image(roi)
            print "postshape", roi.shape
            ret_obj = {}
            ret_obj["id"] = o.id
            ret_obj["points"] = points
            ret_obj["img"] = roi
            ret_obj["time"] = o.header.stamp
            ret_obj["name"] = o.name
            ret_obj["bbox"] = [xmin, ymin, xmax, ymax]
            rois.append(ret_obj)
        defer.returnValue((img, rois))

    def _img_cb(self, image_ros):
        """Add an image to the image cache."""
        self.image = image_ros

        if len(self.image_cache) > 100:
            self.image_cache.popleft()

        self.image_cache.append(image_ros)

    @util.cancellableInlineCallbacks
    def _get_position(self):
        last_odom_msg = yield self._odom_sub.get_next_message()
        defer.returnValue(nt.odometry_to_numpy(last_odom_msg)[0][0])

    @txros.util.cancellableInlineCallbacks
    def _get_bounding_box_2d(self, points_3d_enu, time):
        if self.cam_info is None:
            defer.returnValue((None, None))
        self.camera_model = PinholeCameraModel()
        self.camera_model.fromCameraInfo(self.cam_info)
        points_2d = []
        try:
            trans = yield self.tf_listener.get_transform(self.tf_frame, "/enu", time)
        except Exception:
            defer.returnValue((None, None))
        transformation = trans.as_matrix()

        for point in points_3d_enu:
            point = nt.rosmsg_to_numpy(point)
            point = np.append(point, 1.0)
            t_p = transformation.dot(point)
            if t_p[3] < 1E-15:
                defer.returnValue((None, None))
            t_p[0] /= t_p[3]
            t_p[1] /= t_p[3]
            t_p[2] /= t_p[3]
            t_p = t_p[0:3]
            if t_p[2] < 0:
                defer.returnValue((None, None))

            point_2d = self.camera_model.project3dToPixel(t_p)
            points_2d.append((int(point_2d[0]), int(point_2d[1])))

        xmin, ymin = sys.maxint, sys.maxint
        xmax, ymax = -sys.maxint, -sys.maxint

        for i, point in enumerate(points_2d):
            if point[0] < xmin:
                xmin = point[0]
            if point[0] > xmax:
                xmax = point[0]
            if point[1] < ymin:
                ymin = point[1]
            if point[1] > ymax:
                ymax = point[1]
        defer.returnValue((points_2d, (xmin, ymin, xmax, ymax)))

    @util.cancellableInlineCallbacks
    def _get_closest_image(self, time):
        min_img = None
        for i in range(0, 20):
            min_diff = genpy.Duration(sys.maxint)
            for img in self.image_cache:
                diff = abs(time - img.header.stamp)
                if diff < min_diff:
                    min_diff = diff
                    min_img = img
            print min_diff.to_sec()
            if min_img is not None:
                defer.returnValue(min_img)
            yield self.nh.sleep(.3)

    def _resize_image(self, img):
        h, w, r = img.shape
        if h > w:
            nh = self.MAX_SIZE
            nw = nh * w / h
        else:
            nw = self.MAX_SIZE
            nh = nw * h / w
        img = cv2.resize(img, (nw, nh))
        # return img
        rep = np.ones(nw, dtype=np.int64)
        reph = np.ones(nh, dtype=np.int64)
        emtpy_slots = self.IMAGE_SIZE - nw
        empty_slots_h = self.IMAGE_SIZE - nh
        half_empty_slots = emtpy_slots / 2 + 1
        half_empty_slots_h = empty_slots_h / 2 + 1
        reph[0] = half_empty_slots_h
        reph[-1] = half_empty_slots_h
        rep[0] = half_empty_slots
        rep[-1] = half_empty_slots
        if emtpy_slots % 2 == 1:
            rep[-1] += 1

        if empty_slots_h % 2 == 1:
            reph[-1] += 1

        img = np.repeat(img, reph, axis=0)
        return np.repeat(img, rep, axis=1)
