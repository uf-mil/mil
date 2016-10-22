#!/usr/bin/env python
import roslib
import sys
from collections import deque
from tf import TransformListener
import rospy
import cv2
import tf as tf2
from sensor_msgs.msg import Image, CameraInfo
from geometry_msgs.msg import PointStamped
from cv_bridge import CvBridge, CvBridgeError
import navigator_tools as nt
import numpy as np
from scan_the_code_lib.model_tracker import ModelTracker
from navigator_msgs.srv import ScanTheCodeMission, ScanTheCodeMissionResponse, ObjectDBSingleQuery, ObjectDBSingleQueryRequest
from image_geometry import PinholeCameraModel


class ScanTheCode:

    def __init__(self):

        self.bridge = CvBridge()
        self.image_cache = deque()
        self.model_tracker = ModelTracker()
        self.pinhole_cam = PinholeCameraModel()
        self.last_cam_info = None
        self.tf = TransformListener()
        self.mission_complete = False
        self.colors = []
        self.scan_the_code = None
        self.on = False
        self.image_sub = rospy.Subscriber("/stereo/left/image_raw", Image, self.image_cb, queue_size=10)
        self.cam_info_sub = rospy.Subscriber("/stereo/left/camera_info", CameraInfo, self.info_cb, queue_size=10)

    def image_cb(self, image):
        if len(self.image_cache) > 20:
            self.image_cache.popleft()
        self.image_cache.append(image)

    def info_cb(self, info):
        self.last_cam_info = info

    def get_2d_points(self, points_3d_enu, header):
        points_3d = []
        points_2d = []
        for point in points_3d:
            p = PointStamped()
            p.point = point
            p.header = header
            points_3d.append(self.tf.transformPoint('stereo_left_cam', p))

        K = self.last_cam_info.K
        K = np.array(K).reshape(3, 3)
        for i in range(0, len(points_3d)):
            point = points_3d[i]
            pl = [point.point.x, point.point.y - 2, point.point.z]
            pl = K.dot(pl)
            pl[0] /= pl[2]
            pl[1] /= pl[2]
            points_2d.append((pl[0], pl[1]))

    def start_mission(self):
        # Get the closest image in the deque
        # serv = rospy.ServiceProxy("/database/single", ObjectDBSingleQuery)
        # req = ObjectDBSingleQueryRequest()
        # req.type = 'scan_the_code'
        # scan_the_code = serv(req)
        # scan_the_code = scan_the_code.object
        # points_2d = self.get_2d_points(scan_the_code.points, scan_the_code.header)
        print "sillly"


        img_clone = self.bridge.imgmsg_to_cv2(self.image_cache[len(self.image_cache) - 1], "bgr8").copy()
        cv2.imwrite("train.png", img_clone)
        cv2.waitKey(4000)
        img_clone = self.bridge.imgmsg_to_cv2(self.image_cache[len(self.image_cache) - 1], "bgr8").copy()
        cv2.imwrite("query.png", img_clone)
        # for i, val in enumerate(points_2d):
        #     print val
        #     cv2.circle(img_clone, (int(val[0]), int(val[1])), 3, (255, 255, 255), -1)

        # cv2.imshow("name", img_clone)
        # cv2.waitKey(0)

    def get_bounding_rect(self, contours):
        xmin = 1000
        xmax = 0
        ymin = 1000
        ymax = 0
        for i, cont in enumerate(contours):
            for j, _val in enumerate(cont):
                if(_val[0][0] < xmin):
                    xmin = _val[0][0]
                if(_val[0][0] > xmax):
                    xmax = _val[0][0]
                if(_val[0][1] < ymin):
                    ymin = _val[0][1]
                if(_val[0][1] > ymax):
                    ymax = _val[0][1]
                # print xmin, ymin, xmax, ymax

        return xmin, ymin, xmax, ymax

    def get_non_furthest_points(self, i, points):
        dist_points = []
        for ind, val in enumerate(points):
            if(ind != i):
                diff = np.subtract(points[i], val)
                dist_points.append([np.linalg.norm(diff), points[ind]])

        dist_points = sorted(dist_points, key=lambda x: x[0])

        return dist_points[0][1], dist_points[1][1]

    def geometry_test(self, fp):
        if(len(fp) < 4):
            return False
        for i, p in enumerate(fp):
            p1, p2 = self.get_non_furthest_points(i, fp)
            diff1 = np.subtract(p1, p)[0]
            diff2 = np.subtract(p2, p)[0]
            diff1 /= np.linalg.norm(diff1)
            diff2 /= np.linalg.norm(diff2)

            val = abs(np.dot(diff1, diff2))
            if(val > .05):
                return False

        return True

    def get_salient_points(self, img, rect):
        if(rect[1] == rect[3] or rect[0] == rect[2]):
            return
        roi = img[rect[1]:rect[3], rect[0]:rect[2]]
        gray = cv2.cvtColor(roi, cv2.COLOR_BGR2GRAY)
        fp = cv2.goodFeaturesToTrack(gray, 35, .05, 10.0)  # [, corners[, mask[, blockSize[, useHarrisDetector[, k]]]]]
        if(fp is None):
            return
        # DEBUG
        img_clone = gray.copy()
        for i, val in enumerate(fp):
            cv2.circle(img_clone, tuple(val[0]), 3, (255, 255, 255), -1)

        cv2.imshow("salient_points", img_clone)
        cv2.waitKey(33)

    def mission_status(self, req):
        obs = False
        colors = None
        if(len(self.model_tracker.models) > 0):
            obs = True
        if(self.mission_complete):
            colors = self.model_tracker.colors
        found = self.mission_complete
        return ScanTheCodeMissionResponse(obs, found, colors)

    def activate(self, req):
        if(self.on):
            del self.model_tracker
            self.model_tracker = ModelTracker()
            self.mission_complete = False
            self.colors = []
            self.on = False
        if(not self.on):
            self.on = True
            self.scan_the_code = req.object
            self.start_mission()
        return ScanTheCodeMissionResponse(None, None, None)


def main(args):
    rospy.init_node('model_detector', anonymous=True)
    ic = ScanTheCode()
    status = rospy.Service('/vision/scan_the_code_status', ScanTheCodeMission, ic.mission_status)
    activate = rospy.Service('/vision/scan_the_code_activate', ScanTheCodeMission, ic.activate)

    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down")
    cv2.destroyAllWindows()

if __name__ == '__main__':
    main(sys.argv)
