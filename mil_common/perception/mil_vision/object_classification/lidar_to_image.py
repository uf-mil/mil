import sys
from collections import deque

import cv2
import genpy
import numpy as np
from cv_bridge import CvBridge
from mil_ros_tools import odometry_to_numpy
from nav_msgs.msg import Odometry
from navigator_msgs.srv import ObjectDBQuery, ObjectDBQueryRequest
from navigator_tools import CvDebug
from sensor_msgs.msg import CameraInfo, Image
from txros import tf

___author___ = "Tess Bianchi"

"""
Needs to be refactored to be generic and non depend on navigator.
"""


class LidarToImage:
    def __init__(self, nh, classes=None, dist=50):
        self.MAX_SIZE = 74
        self.IMAGE_SIZE = 100
        self.max_dist = dist
        self.bridge = CvBridge()
        self.nh = nh
        self.image_cache = deque()
        self.pose = None
        self.image = None
        self.classes = classes
        self.cam_info = None
        self.busy = False
        self.c = 0

        self.debug = CvDebug(nh)

    async def init_(self, cam):
        image_sub = "/stereo/right/image_rect_color"
        cam_info = "/stereo/right/camera_info"

        sub = self.nh.subscribe(image_sub, Image, self._img_cb)
        await sub.setup()
        self._database = self.nh.get_service_client("/database/requests", ObjectDBQuery)
        self._odom_sub = self.nh.subscribe(
            "/odom",
            Odometry,
            lambda odom: setattr(self, "pose", odometry_to_numpy(odom)[0]),
        )
        await self._odom_sub.setup()
        self.cam_info_sub = self.nh.subscribe(cam_info, CameraInfo, self._info_cb)
        await self.cam_info_sub.setup()
        self.tf_listener = tf.TransformListener(self.nh)
        return self

    def _info_cb(self, info):
        self.cam_info = info

    def _get_2d_points(self, points_3d):
        # xmin, ymin, zmin = self._get_top_left_point(points_3d)
        points_2d = map(lambda x: self.camera_model.project3dToPixel(x), points_3d)
        return points_2d

    def _get_bounding_rect(self, points_2d, img):
        xmin = np.inf
        xmax = -np.inf
        ymin = np.inf
        ymax = -np.inf
        h, w, r = img.shape
        for i, point in enumerate(points_2d):
            if point[0] < xmin:
                xmin = point[0]
            if point[0] > xmax:
                xmax = point[0]
            if point[1] > ymax:
                ymax = point[1]
            if point[1] < ymin:
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

    async def get_object_rois(self, name=None):
        req = ObjectDBQueryRequest()
        if name is None:
            req.name = "all"
        else:
            req.name = name
        obj = await self._database(req)

        if obj is None or not obj.found:
            return (None, None)
        rois = []
        ros_img = await self._get_closest_image(obj.objects[0].header.stamp)
        if ros_img is None:
            return (None, None)
        img = self.bridge.imgmsg_to_cv2(ros_img, "mono8")
        o = obj.objects[0]

        points_3d = await self.get_3d_points(o)
        points_2d_all = map(lambda x: self.camera_model.project3dToPixel(x), points_3d)
        points_2d = self._get_2d_points(points_3d)
        xmin, ymin, xmax, ymax = self._get_bounding_rect(points_2d, img)
        xmin, ymin, xmax, ymax = int(xmin), int(ymin), int(xmax), int(ymax)
        h, w, r = img.shape
        if (
            xmin < 0
            or xmax < 0
            or xmin > w
            or xmax > w
            or xmax - xmin == 0
            or ymax - ymin == 0
        ):
            return (None, None)
        if ymin < 0:
            ymin = 0
        roi = img[ymin:ymax, xmin:xmax]
        roi = self._resize_image(roi)
        ret_obj = {}
        ret_obj["id"] = o.id
        ret_obj["points"] = points_2d_all
        ret_obj["img"] = roi
        ret_obj["time"] = o.header.stamp
        ret_obj["name"] = o.name
        ret_obj["bbox"] = [xmin, ymin, xmax, ymax]
        rois.append(ret_obj)
        return (img, rois)

    def img_cb(self, image_ros):
        """Add an image to the image cache."""
        self.image = image_ros

        if len(self.image_cache) > 100:
            self.image_cache.popleft()

        self.image_cache.append(image_ros)

    async def get_closest_image(self, time):
        min_img = None
        for i in range(0, 20):
            min_diff = genpy.Duration(sys.maxint)
            for img in self.image_cache:
                diff = abs(time - img.header.stamp)
                if diff < min_diff:
                    min_diff = diff
                    min_img = img
            if min_img is not None:
                return min_img
            await self.nh.sleep(0.3)

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

    async def get_3d_points(self, perc_obj):
        trans = await self.my_tf.get_transform(
            "/stereo_right_cam", "/enu", perc_obj.header.stamp
        )

        stereo_points = []
        for point in perc_obj.points:
            stereo_points.append(np.array([point.x, point.y, point.z, 1]))
        stereo_points = map(lambda x: trans.as_matrix().dot(x), stereo_points)
        points = []
        for p in stereo_points:
            if p[3] < 1e-15:
                raise ZeroDivisionError
            p[0] /= p[3]
            p[1] /= p[3]
            p[2] /= p[3]
            points.append(p[:3])
        return points
